########################## 26/7/9 10:56 최신 버전 #########################
#!/usr/bin/env python3
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry  
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ar_interfaces.msg import LidarScanData
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ultralytics import YOLO
import numpy as np

class FrontCameraNode(Node):
    def __init__(self, blackboard):
        super().__init__('front_camera_node')
        self.blackboard = blackboard
        self.bridge = CvBridge()
        
        # CPU 최적화: PyTorch 쓰레드 수 제한 (선택사항, 필요시 주석 해제)
        # import torch
        # torch.set_num_threads(4) 
        
        self.model = YOLO('yolov8n.pt') 

        # ==========================================
        # 1. QoS 및 멀티스레드 콜백 그룹 분리
        # ==========================================
        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # 와이파이 최적화
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # 밀림 방지
        )
        
        # 이미지 전용, 센서 전용 스레드를 분리하여 서로 간섭하지 않게 함
        self.img_cb_group = MutuallyExclusiveCallbackGroup()
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        ### 퍼블리셔
        # self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', video_qos)
        
        ### 구독자 (콜백 그룹 및 QoS 튜닝 적용)
        self.image_sub = self.create_subscription(
            CompressedImage, '/cam1/image_raw/compressed', self.image_callback, video_qos, callback_group=self.img_cb_group)
        self.human_far_sub = self.create_subscription(
            Bool, '/perception/human_far', self.human_status_cb, 10, callback_group=self.sensor_cb_group)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10, callback_group=self.sensor_cb_group)
        self.scan_sub = self.create_subscription(
            LidarScanData, '/scan/front', self.lidar_callback, qos_profile_sensor_data, callback_group=self.sensor_cb_group)

        ### 변수 선언
        self.last_inference_time = time.time()
        self.inference_interval = 0.2 # 5 FPS (충분함)
        
        # 바운딩 박스 캐싱용 변수 (plot 연산 낭비 방지)
        self.cached_boxes = []


        # 💡 [신규 추가] 사람이 안 보이는 프레임 수를 세는 카운터
        self.miss_count = 0
        self.max_miss_frames = 5  # 0.1초 타이머 기준, 5번(0.5초) 연속 안 보이면 유실로 확정

        
        ### 타이머 (타이머도 별도 스레드에서 돌게 함)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.timer_cb_group)
        self.xmin = None
        self.xmax = None
        self.angles = None
        self.distances = None
        self.get_logger().info("✅ [Data Layer] Front Camera YOLO 노드가 가동되었습니다 (멀티스레드/최적화 적용).")

    # (lidar, human, odom 콜백은 기존과 동일)
    def lidar_callback(self, data):
        if data is not None:
            self.angles = np.array(data.angles)
            self.distances = np.array(data.distances)
    
    def human_status_cb(self, data):
        if data.data is not None:
            self.blackboard.human_far = data.data
            # self.get_logger().info(f"현 상태입니다 {data.   }")
        else:
            pass
        
    def odom_callback(self, msg):
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False

    def image_callback(self, msg):
        current_time = time.time()
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        # ==========================================
        # 2. 0.2초마다 '추론' 수행 및 좌표만 저장
        # ==========================================
        if (current_time - self.last_inference_time) >= self.inference_interval:
            results = self.model(cv_image, classes=[0], verbose=False, conf=0.6, device='cpu')
            self.last_inference_time = current_time

            self.blackboard.is_front_human = False
            self.blackboard.front_obstacle_distance = None
            self.blackboard.is_dynamic_obstacle = False 
            self.cached_boxes = [] # 캐시 초기화

            if results and len(results) > 0 and len(results[0].boxes) > 0:
                bboxes = results[0].boxes.xyxy.cpu().numpy()
                if len(bboxes) > 0:
                    self.xmin, ymin, self.xmax, ymax = map(float, bboxes[0])
                    # 화면 그리기를 위해 박스 좌표 캐싱
                    self.cached_boxes.append((int(self.xmin), int(ymin), int(self.xmax), int(ymax)))
            else:
                self.xmin = -1.0
                self.xmax = -1.0

        # ==========================================
        # 3. 그리기 최적화 (가벼운 cv2.rectangle 사용)
        # 무거운 .plot() 대신 cv2 기본 함수로 덧그리기만 수행 (매 프레임 30fps로 부드럽게 배경 재생)
        # ==========================================
        # for (x1, y1, x2, y2) in self.cached_boxes:
        #     cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        #     cv2.putText(cv_image, "Person", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # try:
        #     # 퍼블리시
        #     result, compressed_img = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
        #     if result:
        #         pub_msg = CompressedImage()
        #         pub_msg.header = msg.header
        #         pub_msg.format = "jpeg"
        #         pub_msg.data = compressed_img.tobytes()
        #         self.image_pub.publish(pub_msg)
        # except Exception as e:
        #     pass
    
    def process_fusion_data(self):
        angle_min_limit = 0.25 * self.xmin + 20.0
        angle_max_limit = 0.25 * self.xmax + 20.0
        mask = (self.angles >= angle_min_limit) & (self.angles <= angle_max_limit)
        target_distances = self.distances[mask]

        if len(target_distances) > 0:
            return np.min(target_distances)
        
        return 200.0
        
    def timer_callback(self):
        # 라이다 데이터 자체가 안 들어온 초기 상태일 때만 무시
        if self.angles is None:
            return
            
        # 💡 [수정됨] YOLO가 사람을 놓쳤을 때의 처리 (인내심 로직)
        if (self.xmin is None) or (self.xmin == -1.0):
            self.miss_count += 1
            
            # 5번 연속으로 안 보였을 때만 진짜로 사라진 것으로 간주하고 200.0 주입!
            if self.miss_count >= self.max_miss_frames:
                self.blackboard.front_obstacle_distance = 200.0
                # 로그를 띄워서 진짜로 200.0이 들어가는 시점을 눈으로 확인하세요.
                # self.get_logger().info("👻 대상 0.5초 이상 유실. 안전거리 200.0cm 주입.")
            return
        # 💡 사람을 다시 찾으면 카운터 초기화
        self.miss_count = 0

        min_dist = self.process_fusion_data()
        
        if min_dist is None:
            min_dist = 200.0 
            
        self.blackboard.front_obstacle_distance = min_dist
        # self.get_logger().info(f"전방 라이다 최소 거리 데이터: {min_dist:.1f}cm")
        # self.get_logger().info(f"전방 라이다 최소 거리 데이터{min_dist}")

def main(args=None):
    rclpy.init(args=args)
    from airport_guide.blackboard import Blackboard
    db = Blackboard()
    node = FrontCameraNode(blackboard=db)
    
    # ==========================================
    # 4. 멀티스레드 실행기 적용
    # ==========================================
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
################ 26/7/9 10:56 이전 버전 
# #!/usr/bin/env python3
# import time
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from nav_msgs.msg import Odometry  
# from std_msgs.msg import Bool
# from cv_bridge import CvBridge
# from rclpy.qos import qos_profile_sensor_data
# from robot_test_msgs.msg import LidarScanData
# from ultralytics import YOLO
# import numpy as np
# # from robot_test.hsv_tracker import HsvTracker

# class FrontCameraNode(Node):
#     def __init__(self, blackboard):
#         super().__init__('front_camera_node')
#         self.blackboard = blackboard
#         self.bridge = CvBridge()
        
#         self.model = YOLO('yolov8n.pt') 

#         ############################################ 데이터 pub, sub 선언 ###########################################
#         ### 욜로 이미지 pub
#         self.image_pub = self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)
        
#         ### 카메라 데이터
#         self.image_sub = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.image_callback, 1)
        
#         ### 후방 캠 사람감지 및 거리에 따른 상태 데이터
#         self.human_far_sub = self.create_subscription(Bool, '/perception/human_far', self.human_status_cb, 10)
        
#         ### odom 데이터
#         self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
#         ### 라이다 데이터
#         self.scan_sub = self.create_subscription(LidarScanData, '/scan/front', self.lidar_callback, qos_profile_sensor_data)

#         ### 기능 분리한 hsv 클래스 객체 생성
#         # self.tracker = HsvTracker()

#         ### 보안장치로 변수 선언 해두기
#         self.last_inference_time = time.time()
#         self.inference_interval = 0.2 
#         self.last_results = None

#         ### timer콜백함수 정의
#         self.timer_period = 0.1
#         self.timer = self.create_timer(self.timer_period, self.timer_callback)
#         self.xmin = None
#         self.xmax = None
#         self.angles = None
#         self.distances = None
#         self.get_logger().info("✅ [Data Layer] Front Camera YOLO 노드가 가동되었습니다.")
        

#     ### 라이다 각도와 거리 데이터 저장
#     def lidar_callback(self, data):
#         if data is not None:
#             self.angles = np.array(data.angles)
#             self.distances = np.array(data.distances)
#             # print(f"각도: {angles}, 거리: {distances}")
#             # self.get_logger().info(f"전방 라이다 거리데이터 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! {self.distances}")
#         else:
#             pass
    
#     def human_status_cb(self, data):

#         if data.data:
#             self.blackboard.human_far = data.data
#         else:
#             pass

#     def odom_callback(self, msg):
#         # [FACTS AREA WRITE]
#         self.blackboard.last_sensor_time = time.time()
#         self.blackboard.sensor_timeout = False

#     def image_callback(self, msg):
#         current_time = time.time()
        
#         try:
#             cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"이미지 변환 실패: {e}")
#             return

#         if (current_time - self.last_inference_time) >= self.inference_interval:
#             self.last_results = self.model(cv_image, classes = [0], verbose=False, conf=0.6, device='cpu')
#             self.last_inference_time = current_time

#             # 🎯 초기화 가드 (FACTS / DERIVED 영역 초기화)
#             self.blackboard.is_front_human = False
#             self.blackboard.front_obstacle_distance = None
#             self.blackboard.is_dynamic_obstacle = False  # Derived 레이어 연동 플래그

#             if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
#                 bboxes = self.last_results[0].boxes.xyxy.cpu().numpy()
                
#                 if len(bboxes) > 0:
#                     self.xmin, ymin, self.xmax, ymax = map(float, bboxes[0])

#             else:
#                 self.xmin = -1.0
#                 self.xmax = -1.0
#                     # if class_id == 0:  # Person 클래스 검출 시
#                     #     # 1) [FACTS AREA WRITE] 원시 데이터 주입
#                     #     self.blackboard.is_front_human = True
#                     #     self.blackboard.front_obstacle_distance = 0.4  # 실험용 물리 거리 강제 매핑
                        
#                     #     # 2) [DERIVED AREA WRITE] Facts 기반 결론 도출 결과 업데이트
#                     #     self.blackboard.is_dynamic_obstacle = True  
                        
#                     #     self.get_logger().info("🚨 [DERIVED] 전방 동적 장애물(사람) 존재 판단 확정.", throttle_duration_sec=1.0)

#         # 이미지 주석 처리 및 토픽 발행 레이어
#         try:
#             if self.last_results and len(self.last_results) > 0:
#                 annotated_frame = self.last_results[0].plot()
#             else:
#                 annotated_frame = cv_image
                
#             if annotated_frame is None:
#                 annotated_frame = cv_image
                
#             result, compressed_img = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
#             if result:
#                 pub_msg = CompressedImage()
#                 pub_msg.header = msg.header
#                 pub_msg.format = "jpeg"
#                 pub_msg.data = compressed_img.tobytes()
#                 self.image_pub.publish(pub_msg)
#         except Exception as pub_err:
#             self.get_logger().error(f"YOLO 이미지 발행 오류: {pub_err}")
    
#     def process_fusion_data(self):

#         ### Bounding Box 좌표를 라이다 각도로 변환
#         ### theta(x) = 0.25 * x + 20
#         angle_min_limit = 0.25 * self.xmin + 20.0
#         angle_max_limit = 0.25 * self.xmax + 20.0

#         ### Numpy 마스킹: 해당 각도 범위 안에 있는 데이터만 True로 필터링
#         ### 조건: angle_min_limit <= angles_deg <= angle_max_limit
#         mask = (self.angles >= angle_min_limit) & (self.angles <= angle_max_limit)

#         ### 마스크를 적용하여 범위 내의 거리 데이터만 추출
#         target_distances = self.distances[mask]
#         # target_angles = self.angles[mask] # 필요시 확인용으로 추출

#         ### 결과 도출
#         if len(target_distances) > 0:
            
#             ### 타겟 영역 안에 200cm 이하의 물체가 존재함! -> 최솟값 추출
#             min_distance_cm = np.min(target_distances)
            
#             ### 최솟값을 가진 데이터의 실제 각도가 궁금하다면 (디버깅용)
#             # min_idx = np.argmin(target_distances)
#             # min_angle_deg = target_angles[min_idx]
            
#             # print(f"[장애물 감지] 픽셀({self.xmin}~{self.xmax}) -> 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역")
#             # print(f"   가장 가까운 장애물: {min_distance_cm:.1f}cm (위치: {min_angle_deg:.1f}°)")
            
#             return min_distance_cm
#         else:
#             ### Bounding Box 영역 안에 있긴 하지만, 거리가 200cm 밖이라 
#             ### 전처리 과정에서 날아갔거나 아예 텅 빈 허공인 경우
#             # print(f" [안전] 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역 내 200cm 이하 장애물 없음")
            
#             return None ### 혹은 안전을 뜻하는 기본값 반환 (예: 200.0)
        
#     def timer_callback(self):
#         ### 예외처리 처음 시작시 데이터가 없을 때
#         if (self.xmin is None) or (self.angles is None):
#             return
#         min_dist = self.process_fusion_data()
#         self.blackboard.front_obstacle_distance = min_dist
#         self.get_logger().info(f"현재 전방 장애물과의 거리 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! {self.blackboard.front_obstacle_distance}")


# def main(args=None):
#     rclpy.init(args=args)
#     from airport_guide.blackboard import Blackboard
#     db = Blackboard()
#     node = FrontCameraNode(blackboard=db)
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
    
# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import CompressedImage
# from nav_msgs.msg import Odometry  
# from std_msgs.msg import Bool
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import time
# import cv2


# class FrontCameraNode(Node):
#     def __init__(self, blackboard):
#         super().__init__('front_camera_node')
#         self.blackboard = blackboard
#         self.bridge = CvBridge()
        
#         self.model = YOLO('yolov8n.pt') 

#         self.image_pub = self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)
#         self.image_sub = self.create_subscription(
#             CompressedImage,
#             '/cam1/image_raw/compressed', 
#             self.image_callback,
#             10
#         )

#         self.human_far_sub = self.create_subscription(
#             Bool,
#             '/perception/human_far',
#             self.human_status_cb,
#             10
#         )


#         self.odom_sub = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
        
#         self.last_inference_time = time.time()
#         self.inference_interval = 0.2 
#         self.last_results = None

#         self.get_logger().info("✅ [Data Layer] Front Camera YOLO 노드가 가동되었습니다.")


#     def human_status_cb(self, data):
#         self.blackboard.human_far = data.data

#     def odom_callback(self, msg):
#         # [FACTS AREA WRITE]
#         self.blackboard.last_sensor_time = time.time()
#         self.blackboard.sensor_timeout = False

#     def image_callback(self, msg):
#         current_time = time.time()
        
#         try:
#             cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"이미지 변환 실패: {e}")
#             return

#         if (current_time - self.last_inference_time) >= self.inference_interval:
#             self.last_results = self.model(cv_image, verbose=False, conf=0.6, device='cpu')
#             self.last_inference_time = current_time

#             # 🎯 초기화 가드 (FACTS / DERIVED 영역 초기화)
#             self.blackboard.is_front_human = False
#             self.blackboard.front_obstacle_distance = 10.0
#             self.blackboard.is_dynamic_obstacle = False  # Derived 레이어 연동 플래그

#             if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
#                 for box in self.last_results[0].boxes:
#                     class_id = int(box.cls[0])
                    
#                     if class_id == 0:  # Person 클래스 검출 시
#                         # 1) [FACTS AREA WRITE] 원시 데이터 주입
#                         self.blackboard.is_front_human = True
#                         self.blackboard.front_obstacle_distance = 0.4  # 실험용 물리 거리 강제 매핑
                        
#                         # 2) [DERIVED AREA WRITE] Facts 기반 결론 도출 결과 업데이트
#                         self.blackboard.is_dynamic_obstacle = True  
                        
#                         self.get_logger().info("🚨 [DERIVED] 전방 동적 장애물(사람) 존재 판단 확정.", throttle_duration_sec=1.0)

#         # 이미지 주석 처리 및 토픽 발행 레이어
#         try:
#             if self.last_results and len(self.last_results) > 0:
#                 annotated_frame = self.last_results[0].plot()
#             else:
#                 annotated_frame = cv_image
                
#             if annotated_frame is None:
#                 annotated_frame = cv_image
                
#             result, compressed_img = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
#             if result:
#                 pub_msg = CompressedImage()
#                 pub_msg.header = msg.header
#                 pub_msg.format = "jpeg"
#                 pub_msg.data = compressed_img.tobytes()
#                 self.image_pub.publish(pub_msg)
#         except Exception as pub_err:
#             self.get_logger().error(f"YOLO 이미지 발행 오류: {pub_err}")


# def main(args=None):
#     rclpy.init(args=args)
#     from airport_guide.blackboard import Blackboard
#     db = Blackboard()
#     node = FrontCameraNode(blackboard=db)
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()