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
            
        # [수정됨] YOLO가 사람을 놓쳤을 때의 처리 (인내심 로직)
        if (self.xmin is None) or (self.xmin == -1.0):
            self.miss_count += 1
            
            # 5번 연속으로 안 보였을 때만 진짜로 사라진 것으로 간주하고 200.0 주입!
            if self.miss_count >= self.max_miss_frames:
                self.blackboard.front_obstacle_distance = 200.0
            return
        # 사람을 다시 찾으면 카운터 초기화
        self.miss_count = 0

        min_dist = self.process_fusion_data()
        
        if min_dist is None:
            min_dist = 200.0 
            
        self.blackboard.front_obstacle_distance = min_dist

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
