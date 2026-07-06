#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry  
from std_msgs.msg import Bool
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

        self.image_pub = self.image_pub = self.create_publisher(CompressedImage, '/yolo/image_raw/compressed', 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/cam1/image_raw/compressed', 
            self.image_callback,
            10
        )

        self.human_far_sub = self.create_subscription(
            Bool,
            '/perception/human_far',
            self.human_status_cb,
            10
        )


        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.last_inference_time = time.time()
        self.inference_interval = 0.2 
        self.last_results = None

        self.get_logger().info("✅ [Data Layer] Front Camera YOLO 노드가 가동되었습니다.")


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
            self.last_results = self.model(cv_image, verbose=False, conf=0.6, device='cpu')
            self.last_inference_time = current_time

            # 🎯 초기화 가드 (FACTS / DERIVED 영역 초기화)
            self.blackboard.is_front_human = False
            self.blackboard.front_obstacle_distance = 10.0
            self.blackboard.is_dynamic_obstacle = False  # Derived 레이어 연동 플래그

            if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
                for box in self.last_results[0].boxes:
                    class_id = int(box.cls[0])
                    
                    if class_id == 0:  # Person 클래스 검출 시
                        # 1) [FACTS AREA WRITE] 원시 데이터 주입
                        self.blackboard.is_front_human = True
                        self.blackboard.front_obstacle_distance = 0.4  # 실험용 물리 거리 강제 매핑
                        
                        # 2) [DERIVED AREA WRITE] Facts 기반 결론 도출 결과 업데이트
                        self.blackboard.is_dynamic_obstacle = True  
                        
                        self.get_logger().info("🚨 [DERIVED] 전방 동적 장애물(사람) 존재 판단 확정.", throttle_duration_sec=1.0)

        # 이미지 주석 처리 및 토픽 발행 레이어
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
            self.get_logger().error(f"YOLO 이미지 발행 오류: {pub_err}")


def main(args=None):
    rclpy.init(args=args)
    from airport_guide.blackboard import Blackboard
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