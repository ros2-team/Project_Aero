import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # 1. 통신 인프라 설정
        
        # 2. AI 및 비전 도구 초기화
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') # VRAM 회피용 CPU 강제 할당 적용
        
        # 3. 추적 상태 변수
        self.locked_id = -1
        self.last_seen_time = 0.0
        self.MIN_AREA = 15000

        # [신규] 후보군 검증(2초 대기)을 위한 변수
        self.candidate_id = -1
        self.candidate_first_seen = 0.0
        self.LOCK_ON_DELAY = 2.0  # 2초
        
        # [신규] 히스토그램 Re-ID 및 타겟 유실 타이머
        self.target_hist = None
        self.HIST_THRESHOLD = 0.6  # 유사도 기준 (0~1)
        self.LOST_TIMEOUT = 3.0    # 3초
        
        # [신규] 부드러운 거리 제어를 위한 면적 스무딩(EMA)
        self.smoothed_area = 0.0
        self.ALPHA = 0.2  # 스무딩 계수 (작을수록 변화에 둔감하여 부드러움)

        # """ROS2 Pub/Sub 통신망 설정 (초기화 분리)"""
        self.image_sub = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.image_callback, 1)
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_data', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/cam_rear/yolo/compressed', 1)

    # [콜백 함수]
    def image_callback(self, msg):
        # 1. 이미지 변환
        frame = self._convert_ros_to_cv2(msg)
        if frame is None:
            return
            
        # 2. YOLO 추론
        results = self._run_yolo_tracking(frame)
        
        # 3. 타겟 정보 추출 (Auto Lock-on 포함)
        target_info = self._extract_target_data(results)
        print("타겟 정보 !! ", target_info[0])

        # 디버깅용 화면 출력 (실전에서는 주석 처리하여 리소스 절약)
        # cv2.imshow("Tracking", annotated_frame)
        # cv2.waitKey(1)

        # # 4. 제어 노드로 결과 전송
        annotated_frame = results[0].plot()
        self.image_publish(annotated_frame)
        self._publish_target_data(*target_info)

        ####################### 이미지 변환 ##########################
    def _convert_ros_to_cv2(self, msg):
        try:
            return self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None
        
        ############### yolo 모델 정의 ##################
    def _run_yolo_tracking(self, frame):
        return self.model.track(
            frame, tracker="bytetrack.yaml", classes=[0], 
            persist=True, device='cpu', verbose=False
        )
    
        ################### ROS2 시간을 초(초) 단위 float로 반환 ######################
    def _get_current_time(self):
        return self.get_clock().now().nanoseconds / 1e9

        ########################## 추론 결과를 분석하여 타겟의 ID, X좌표, 면적을 계산 및 반환 ############################
    def _extract_target_data(self, results):
        if results[0].boxes is None or results[0].boxes.id is None:
            return self._handle_empty_frame()

        # 데이터 분리 (x,y), id, 바운딩박스 면적 크기, 원본 이미지
        boxes = results[0].boxes.xyxy.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in boxes]
        img = results[0].orig_img  # 원본 이미지 (히스토그램 추출용)

        # 1. 락온된 타겟이 현재 화면에 있는 경우 (최우선 순위 유지)
        if self.locked_id in track_ids:
            return self._process_locked_target(track_ids, boxes, areas, img)

        # 2. 타겟을 잃어버렸지만 리셋 전인 경우 -> 히스토그램 Re-ID 시도
        if self.locked_id != -1:
            reid_success = self._attempt_reid(track_ids, boxes, img)
            if reid_success:
                return self._process_locked_target(track_ids, boxes, areas, img)
            else:
                return self._handle_target_lost()

        # 3. 락온된 타겟이 없는 초기 상태 -> 새로운 후보군 탐색 및 검증 (2초 룰)
        self._evaluate_candidate(track_ids, areas)
        return -1.0, 0.0, 0.0

        ################ 현재 추적중인 타겟의 id, 바운딩박스 가운데 x좌표, 바운딩 박스 크기 계산 ######################
    def _process_locked_target(self, track_ids, boxes, areas, img):
        idx = track_ids.index(self.locked_id)
        box = boxes[idx]
        current_area = areas[idx]
        
        # 타겟 관측 시간 갱신
        self.last_seen_time = self._get_current_time()
        
        # 지속적인 히스토그램 업데이트 (조명 변화 적응)
        self.target_hist = self._get_hsv_histogram(img, box)

        # 면적 스무딩 (급격한 정지/출발 방지)
        if self.smoothed_area == 0.0:
            self.smoothed_area = current_area
        else:
            self.smoothed_area = (self.ALPHA * current_area) + ((1 - self.ALPHA) * self.smoothed_area)

        center_x = (box[0] + box[2]) / 2.0
        
        return float(self.locked_id), center_x, float(self.smoothed_area)

        ###################### 화면에 아무 객체도 없을 때의 처리 ######################
    def _handle_empty_frame(self):
        if self.locked_id != -1:
            return self._handle_target_lost()
        return -1.0, 0.0, 0.0

        #################### 타겟 유실 시 타임아웃 체크 및 리셋 처리 ###################
    def _handle_target_lost(self):
        time_since_lost = self._get_current_time() - self.last_seen_time
        
        if time_since_lost > self.LOST_TIMEOUT:
            self.get_logger().warn("타겟을 놓쳤습니다. id 초기화")
            self._reset_target_state()
        else:
            self.get_logger().info(f"타겟 가려짐 대기중 .. ({time_since_lost:.1f}s / {self.LOST_TIMEOUT}s)")
            
        return -1.0, 0.0, 0.0

        ##################### 추적 상태 초기화 #########################
    def _reset_target_state(self):
        self.locked_id = -1
        self.target_hist = None
        self.smoothed_area = 0.0
        self.candidate_id = -1

        ####################### 가장 큰 객체를 찾아 2초 이상 유지되는지 검증 후 락온 ########################
    def _evaluate_candidate(self, track_ids, areas):
        max_area_idx = areas.index(max(areas))
        largest_id = track_ids[max_area_idx]
        largest_area = areas[max_area_idx]
        
        # 너무 먼 대상은 제외
        if largest_area < self.MIN_AREA:
            self.candidate_id = -1
            return

        current_time = self._get_current_time()

        # 새로운 후보 발견
        if self.candidate_id != largest_id:
            self.candidate_id = largest_id
            self.candidate_first_seen = current_time
            self.get_logger().info(f"새 후보 포착 ID: {largest_id}, 검증 시작...")
        
        # 동일 후보가 지정된 시간(2초) 이상 유지됨
        elif current_time - self.candidate_first_seen >= self.LOCK_ON_DELAY:
            self.locked_id = self.candidate_id
            self.last_seen_time = current_time
            self.get_logger().info(f"타겟 추적 완료 ID: {self.locked_id}")

        ########################### 현재 화면의 객체들과 저장된 히스토그램을 비교하여 사람 예측 ##############################
    def _attempt_reid(self, track_ids, boxes, img): # (x,y)좌표, id, 원본 이미지 받음
        if self.target_hist is None:
            return False

        # 비교 변수 선언 점수 / id
        best_match_score = 0.0
        best_match_id = -1

        # (0, x1), (0, x2), (2, y1), (3, y2)
        for i, box in enumerate(boxes):
            candidate_hist = self._get_hsv_histogram(img, box)
            if candidate_hist is None:
                continue
            
            # 히스토그램 교차(Intersection) 비교 (1.0에 가까울수록 동일)
            score = cv2.compareHist(self.target_hist, candidate_hist, cv2.HISTCMP_CORREL)
            
            if score > best_match_score:
                best_match_score = score
                best_match_id = track_ids[i]

        if best_match_score > self.HIST_THRESHOLD:
            self.get_logger().info(f" 색 인식 성공! 이전 ID: {self.locked_id} -> 새 ID: {best_match_id} (유사도: {best_match_score:.2f})")
            self.locked_id = best_match_id
            return True
            
        return False

        ###################### 바운딩 박스 중심 50% 영역의 HSV 색상 히스토그램 추출 #########################
    def _get_hsv_histogram(self, img, box):
        x1, y1, x2, y2 = map(int, box)
        w, h = x2 - x1, y2 - y1
        
        # 너무 작은 박스는 예외 처리
        if w < 10 or h < 10:
            return None

        # 배경 노이즈 제거를 위해 중심 50% 영역만 크롭 (Core Crop)
        cx1, cx2 = x1 + int(w * 0.25), x2 - int(w * 0.25)
        cy1, cy2 = y1 + int(h * 0.25), y2 - int(h * 0.25)
        
        crop_img = img[cy1:cy2, cx1:cx2]
        
        # HSV 변환 후 H(색상), S(채도) 채널만 사용하여 조명 변화에 강건하게 설정
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [50, 50], [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        
        return hist
    
    def _publish_target_data(self, target_id, center_x, area):
        # """추출된 데이터를 Float32MultiArray 형태로 조립하여 퍼블리시"""
        msg = Float32MultiArray()
        msg.data = [target_id, float(center_x), float(area)]
        self.target_pub.publish(msg)

    def image_publish(self, data):
        try:
            annotated_frame = data
            result, compressed_img = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if result:    
                msg = CompressedImage()
                msg.header = msg.header
                msg.format = "jpeg"
                msg.data = compressed_img.tobytes()

                self.image_pub.publish(msg)
        except Exception as pub_err:
            self.get_logger().info(f"yolo 이미지 토픽 에러 {pub_err}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




    # [내부 메서드] - 기능별 분리된 핵심 로직


    # def _extract_target_data(self, results):
    #     # """추론 결과를 분석하여 타겟의 ID, X좌표, 면적을 계산 및 반환"""
    #     default_target_id = -1.0
    #     center_x = 0.0
    #     area = 0.0

    #     if results[0].boxes is None or results[0].boxes.id is None:
    #         return default_target_id, center_x, area

    #     # CPU 연산 및 numpy 배열 변환
    #     boxes = results[0].boxes.xyxy.cpu().numpy()
    #     track_ids = results[0].boxes.id.int().cpu().tolist()
    #     areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in boxes]

    #     # [기능 A] 타겟이 설정되지 않은 상태 -> Auto Lock-on 수행
    #     if self.locked_id == -1:
    #         self._attempt_lock_on(track_ids, areas)

    #     # [기능 B] 현재 화면에 락온된 타겟이 존재하는지 확인 후 데이터 추출
    #     if self.locked_id in track_ids:
    #         idx = track_ids.index(self.locked_id)
    #         box = boxes[idx]
            
    #         center_x = (box[0] + box[2]) / 2.0
    #         area = areas[idx]
    #         target_id = float(self.locked_id)
    #         return target_id, center_x, float(area)
        
    #     else:
    #         # 2. 타겟이 화면에서 사라졌을 때 (스스로 판단)
    #         if self.locked_id != -1:
    #             time_since_lost = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
                
    #             if time_since_lost > 3.0: # 3초(절대 시간) 초과 시 스스로 리셋!
    #                 self.get_logger().warn("🚨 Target Lost Timeout! Auto-resetting.")
    #                 self.locked_id = -1
    #             else:
    #                 self.get_logger().info(f"⚠️ Target Occluded... Waiting ({time_since_lost:.1f}s/1.0s)")
        
    #         return -1.0, 0.0, 0.0 # 타겟 안 보이니까 -1 리턴

    # def _attempt_lock_on(self, track_ids, areas):
    #     # """가장 크고 가까운 객체를 찾아 Lock-on 수행"""
    #     max_area_idx = areas.index(max(areas))
    #     if areas[max_area_idx] > self.MIN_AREA:
    #         self.locked_id = track_ids[max_area_idx]
    #         self.get_logger().info(f" 타겟 고정 ID: {self.locked_id}")