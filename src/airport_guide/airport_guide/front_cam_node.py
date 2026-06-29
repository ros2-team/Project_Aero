#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry  # 🎯 [추가] 오도메트리 메시지 타입 임포트
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
import cv2

class FrontCameraNode(Node):
    def __init__(self, blackboard):
        super().__init__('front_camera_node')
        self.blackboard = blackboard
        self.bridge = CvBridge()
        
        self.model = YOLO('yolov8n.pt') 

        # 발행하는 토픽과 구독하는 토픽의 이름을 명확히 분리하여 무한 루프를 방지합니다.
        self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)

        # 실제 카메라가 장치 드라이버로부터 쏴주는 원본 압축 이미지 토픽 명으로 지정해야 합니다.
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/cam1/image_raw/compressed', 
            self.image_callback,
            10
        )
        
        # 🎯 [추가] 로봇 기저 레이어의 오도메트리 토픽을 구독합니다.
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.stop_class_ids = [0, 15, 16, 17, 18, 19]
        self.avoid_class_ids = [2, 5, 7]
        
        self.last_inference_time = time.time()
        self.inference_interval = 0.2 
        self.last_results = None

        self.get_logger().info("✅ 전방 카메라, YOLO 스케줄러 및 Odom 감시 통합 노드가 가동되었습니다.")

    # 🎯 [추가] 오도메트리 토픽이 들어올 때마다 공유 블랙보드의 타임스탬프를 실시간 최신화합니다.
    def odom_callback(self, msg):
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
            self.last_results = self.model(cv_image, verbose=False, conf=0.6, device='cpu')
            self.last_inference_time = current_time

            self.blackboard.is_front_human = False
            self.blackboard.front_obstacle_distance = 10.0
            human_count = 0

            if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
                for box in self.last_results[0].boxes:
                    class_id = int(box.cls[0])
                    
                    if class_id == 0:  # Person 클래스
                        human_count += 1
                        self.blackboard.is_front_human = True
                        self.blackboard.front_obstacle_distance = 0.4  # 실험용 강제 근접 거리 세팅
                        self.get_logger().info(f"🚨 [EMERGENCY] 전방 인물 감지!! 즉시 정지 플래그 작동: 현재 {human_count}명", throttle_duration_sec=0.5)

        try:
            if self.last_results and len(self.last_results) > 0:
                annotated_frame = self.last_results[0].plot()
            else:
                annotated_frame = cv_image
                
            if annotated_frame is None:
                annotated_frame = cv_image
                
            result, compressed_img = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            if result:
                pub_msg = CompressedImage()
                pub_msg.header = msg.header
                pub_msg.format = "jpeg"
                pub_msg.data = compressed_img.tobytes()
                self.image_pub.publish(pub_msg)
        except Exception as pub_err:
            self.get_logger().error(f"YOLO 압축 이미지 토픽 발행 실패: {pub_err}")


def main(args=None):
    rclpy.init(args=args)
    
    class DummyBlackboard:
        def __init__(self):
            self.is_front_human = False
            self.front_obstacle_distance = 10.0
            self.last_sensor_time = time.time()
            self.sensor_timeout = False
            
    db = DummyBlackboard()
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