#!/usr/bin/env python3
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry  
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ar_interfaces.msg import LidarScanData
from ultralytics import YOLO
import numpy as np
from perception.hsv_tracker import HsvTracker

class FrontCameraNode(Node):
    def __init__(self, blackboard):
        super().__init__('front_camera_node')
        self.blackboard = blackboard
        self.bridge = CvBridge()
        
        self.model = YOLO('yolov8n.pt') 

        ############################################ 데이터 pub, sub 선언 ###########################################
        ### 욜로 이미지 pub
        # self.image_pub = self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)
        
        ### 카메라 데이터
        self.image_sub = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.image_callback, 10)
        
        ### 후방 캠 사람감지 및 거리에 따른 상태 데이터
        self.human_far_sub = self.create_subscription(Bool, '/perception/human_far', self.human_status_cb, 10)
        
        ### odom 데이터
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        ### 라이다 데이터
        self.scan_sub = self.create_subscription(LidarScanData, '/scan/front', self.lidar_callback, qos_profile_sensor_data)

        ### 기능 분리한 hsv 클래스 객체 생성
        self.tracker = HsvTracker()

        ### 보안장치로 변수 선언 해두기
        self.last_inference_time = time.time()
        self.inference_interval = 0.2 
        self.last_results = None

        ### timer콜백함수 정의
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.xmin = None
        self.xmax = None
        self.angles = None
        self.distances = None
        self.get_logger().info("✅ [Data Layer] Front Camera YOLO 노드가 가동되었습니다.")
        

    ### 라이다 각도와 거리 데이터 저장
    def lidar_callback(self, data):
        if data is not None:
            self.angles = np.array(data.angles)
            self.distances = np.array(data.distances)
            # print(f"각도: {angles}, 거리: {distances}")
        else:
            pass
    
    def human_status_cb(self, data):
        self.blackboard.human_far = data.data

    def odom_callback(self, msg):
        # [FACTS AREA WRITE]
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False

    def image_callback(self, msg):
        current_time = time.time()
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        if (current_time - self.last_inference_time) >= self.inference_interval:
            self.last_results = self.model(cv_image, classes = [0], verbose=False, conf=0.6, device='cpu')
            self.last_inference_time = current_time

            # 🎯 초기화 가드 (FACTS / DERIVED 영역 초기화)
            self.blackboard.is_front_human = False
            self.blackboard.front_obstacle_distance = 10.0
            self.blackboard.is_dynamic_obstacle = False  # Derived 레이어 연동 플래그

            if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
                bboxes = self.last_results[0].boxes.xyxy.cpu().numpy()
                
                if len(bboxes) > 0:
                    target_bbox = self.tracker.get_target_bbox(cv_image, bboxes)

                    if target_bbox is not None:
                        self.xmin, ymin, self.xmax, ymax = map(float, target_bbox)

            else:
                self.xmin = -1.0
                self.xmax = -1.0
                    # if class_id == 0:  # Person 클래스 검출 시
                    #     # 1) [FACTS AREA WRITE] 원시 데이터 주입
                    #     self.blackboard.is_front_human = True
                    #     self.blackboard.front_obstacle_distance = 0.4  # 실험용 물리 거리 강제 매핑
                        
                    #     # 2) [DERIVED AREA WRITE] Facts 기반 결론 도출 결과 업데이트
                    #     self.blackboard.is_dynamic_obstacle = True  
                        
                    #     self.get_logger().info("🚨 [DERIVED] 전방 동적 장애물(사람) 존재 판단 확정.", throttle_duration_sec=1.0)

        # 이미지 주석 처리 및 토픽 발행 레이어
        # try:
        #     if self.last_results and len(self.last_results) > 0:
        #         annotated_frame = self.last_results[0].plot()
        #     else:
        #         annotated_frame = cv_image
                
        #     if annotated_frame is None:
        #         annotated_frame = cv_image
                
        #     result, compressed_img = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
        #     if result:
        #         pub_msg = CompressedImage()
        #         pub_msg.header = msg.header
        #         pub_msg.format = "jpeg"
        #         pub_msg.data = compressed_img.tobytes()
        #         self.image_pub.publish(pub_msg)
        # except Exception as pub_err:
        #     self.get_logger().error(f"YOLO 이미지 발행 오류: {pub_err}")
    
    def process_fusion_data(self):

        ### Bounding Box 좌표를 라이다 각도로 변환
        ### theta(x) = 0.25 * x + 20
        angle_min_limit = 0.25 * self.xmin + 20.0
        angle_max_limit = 0.25 * self.xmax + 20.0

        ### Numpy 마스킹: 해당 각도 범위 안에 있는 데이터만 True로 필터링
        ### 조건: angle_min_limit <= angles_deg <= angle_max_limit
        mask = (self.angles >= angle_min_limit) & (self.angles <= angle_max_limit)

        ### 마스크를 적용하여 범위 내의 거리 데이터만 추출
        target_distances = self.distances[mask]
        # target_angles = self.angles[mask] # 필요시 확인용으로 추출

        ### 결과 도출
        if len(target_distances) > 0:
            
            ### 타겟 영역 안에 200cm 이하의 물체가 존재함! -> 최솟값 추출
            min_distance_cm = np.min(target_distances)
            
            ### 최솟값을 가진 데이터의 실제 각도가 궁금하다면 (디버깅용)
            # min_idx = np.argmin(target_distances)
            # min_angle_deg = target_angles[min_idx]
            
            # print(f"[장애물 감지] 픽셀({self.xmin}~{self.xmax}) -> 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역")
            # print(f"   가장 가까운 장애물: {min_distance_cm:.1f}cm (위치: {min_angle_deg:.1f}°)")
            
            return min_distance_cm
        else:
            ### Bounding Box 영역 안에 있긴 하지만, 거리가 200cm 밖이라 
            ### 전처리 과정에서 날아갔거나 아예 텅 빈 허공인 경우
            # print(f" [안전] 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역 내 200cm 이하 장애물 없음")
            
            return None ### 혹은 안전을 뜻하는 기본값 반환 (예: 200.0)
        
    def timer_callback(self):
        ### 예외처리 처음 시작시 데이터가 없을 때
        if (self.xmin is None) or (self.angles is None):
            return
        min_dist = self.process_fusion_data()
        self.blackboard.front_obstacle_distance = min_dist


def main(args=None):
    rclpy.init(args=args)
    from behavior_tree.blackboard import Blackboard
    db = Blackboard()
    node = FrontCameraNode(blackboard=db)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()