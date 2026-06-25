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
        self.MIN_AREA = 15000

        # """ROS2 Pub/Sub 통신망 설정 (초기화 분리)"""
        self.image_sub = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.image_callback, 10)
        self.target_pub = self.create_publisher(Float32MultiArray, '/target_data', 10)

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
        print("타겟 정보 !! ", np.round(target_info[2]))
        print(type(target_info))
        # 디버깅용 화면 출력 (실전에서는 주석 처리하여 리소스 절약)
        annotated_frame = results[0].plot()
        cv2.imshow("Tracking", annotated_frame)
        cv2.waitKey(1)
        # 4. 제어 노드로 결과 전송
        self._publish_target_data(*target_info)

    # [내부 메서드] - 기능별 분리된 핵심 로직
    def _convert_ros_to_cv2(self, msg):
        try:
            return self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None

    def _run_yolo_tracking(self, frame):
        return self.model.track(
            frame, tracker="bytetrack.yaml", classes=[0], 
            persist=True, device='cpu', verbose=False
        )

    def _extract_target_data(self, results):
        # """추론 결과를 분석하여 타겟의 ID, X좌표, 면적을 계산 및 반환"""
        default_target_id = -1.0
        center_x = 0.0
        area = 0.0

        if results[0].boxes is None or results[0].boxes.id is None:
            return default_target_id, center_x, area

        # CPU 연산 및 numpy 배열 변환
        boxes = results[0].boxes.xyxy.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in boxes]

        # [기능 A] 타겟이 설정되지 않은 상태 -> Auto Lock-on 수행
        if self.locked_id == -1:
            self._attempt_lock_on(track_ids, areas)

        # [기능 B] 현재 화면에 락온된 타겟이 존재하는지 확인 후 데이터 추출
        if self.locked_id in track_ids:
            idx = track_ids.index(self.locked_id)
            box = boxes[idx]
            
            center_x = (box[0] + box[2]) / 2.0
            area = areas[idx]
            target_id = float(self.locked_id)
            return target_id, center_x, float(area)
        
        else:
            # 2. 타겟이 화면에서 사라졌을 때 (스스로 판단)
            if self.locked_id != -1:
                time_since_lost = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
                
                if time_since_lost > 3.0: # 3초(절대 시간) 초과 시 스스로 리셋!
                    self.get_logger().warn("🚨 Target Lost Timeout! Auto-resetting.")
                    self.locked_id = -1
                else:
                    self.get_logger().info(f"⚠️ Target Occluded... Waiting ({time_since_lost:.1f}s/1.0s)")
        
            return -1.0, 0.0, 0.0 # 타겟 안 보이니까 -1 리턴

    def _attempt_lock_on(self, track_ids, areas):
        # """가장 크고 가까운 객체를 찾아 Lock-on 수행"""
        max_area_idx = areas.index(max(areas))
        if areas[max_area_idx] > self.MIN_AREA:
            self.locked_id = track_ids[max_area_idx]
            self.get_logger().info(f" 타겟 고정 ID: {self.locked_id}")

    def _publish_target_data(self, target_id, center_x, area):
        # """추출된 데이터를 Float32MultiArray 형태로 조립하여 퍼블리시"""
        msg = Float32MultiArray()
        msg.data = [target_id, float(center_x), float(area)]
        self.target_pub.publish(msg)

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