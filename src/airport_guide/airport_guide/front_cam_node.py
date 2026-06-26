import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import time
import cv2

class FrontCameraNode(Node):
    def __init__(self, blackboard):
        super().__init__('front_camera_node')
        self.blackboard = blackboard
        self.bridge = CvBridge()
        
        # 팩트: 인텔 CPU 환경이라면 나중에 'yolov8n_openvino_model/' 형태로 변환해 로드하면 훨씬 빨라집니다.
        self.model = YOLO('yolov8n.pt') 

        # 압축 이미지 토픽 발행자 설정
        self.image_pub = self.create_publisher(CompressedImage, '/cam2/yolo_image/compressed', 10)

        # 원본 이미지 토픽 구독 설정
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/cam2/image_raw/compressed', 
            self.image_callback,
            10
        )
        
        self.stop_class_ids = [0, 15, 16, 17, 18, 19]
        self.avoid_class_ids = [2, 5, 7]
        
        # 🎯 [CPU 과열 방지 핵심 변수] 최종 YOLO 연산 수행 타임스탬프 기록
        self.last_inference_time = time.time()
        # 🎯 [추론 주기 설정] 0.2초에 1번만 YOLO 돌리기 (약 5 FPS 제한 -> CPU 부하 극적인 감소 효과)
        self.inference_interval = 0.2 
        
        # 🎯 최신 프레임 결과 보존용 변수 (YOLO를 건너뛰는 프레임에서는 이전 바운딩 박스를 그대로 그림)
        self.last_results = None

        self.get_logger().info("✅ 전방 카메라 및 YOLO 프레임 스케줄러 노드가 가동되었습니다.")

    def image_callback(self, msg):
        current_time = time.time()
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # 🎯 [핵심 가드] 설정한 인터벌(0.2초)이 지나지 않았다면 무거운 YOLO 연산을 스킵합니다.
        if (current_time - self.last_inference_time) >= self.inference_interval:
            # CPU 전력 폭주를 막기 위해 명시적으로 device='cpu'를 지정하여 추론을 실행합니다.
            self.last_results = self.model(cv_image, verbose=False, conf=0.6, device='cpu')
            self.last_inference_time = current_time # 타임스탬프 갱신

        # 블랙보드 플래그 및 카운터 초기화
        self.blackboard.is_front_human = False
        self.blackboard.front_obstacle_distance = 10.0
        human_count = 0

        # 유효한 추론 결과가 존재하는 경우 상태 판독 시작
        if self.last_results and len(self.last_results) > 0 and len(self.last_results[0].boxes) > 0:
            for box in self.last_results[0].boxes:
                class_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = self.model.names[class_id]
                
                if class_id == 0:
                    human_count += 1
                    self.blackboard.is_front_human = True
                    self.blackboard.front_obstacle_distance = 0.4  
                    self.get_logger().info(f"🔍 [YOLO 인물 감지]: {name}({conf*100:.1f}%) [현재 인원]: {human_count}", throttle_duration_sec=1.0)

        # OpenCV 이미지 압축 및 ROS2 토픽 토크 발행 레이어
        try:
            # YOLO를 스킵한 프레임이라도 이전의 바운딩 박스 아웃라인 결과를 유지하여 시각화 품질을 방어합니다.
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

        # 하드웨어 무결성 플래그 터치
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False

def main(args=None):
    rclpy.init(args=args)
    class DummyBlackboard:
        def __init__(self):
            self.is_front_human = False
            self.front_obstacle_distance = 10.0
            self.last_sensor_time = 0.0
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