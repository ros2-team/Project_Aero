import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from ultralytics import YOLO
from perception.hsv_tracker import HsvTracker

class SimpleYoloDetector(Node):
    def __init__(self):
        super().__init__('simple_yolo_detector')
        
        ### 구독(Sub): 카메라 압축 이미지 수신
        self.sub_img = self.create_subscription(CompressedImage, '/cam2/image_raw/compressed', self.image_callback, 10)
        
        ### 발행(Pub): X좌표 3개(xmin, center_x, xmax)를 보낼 토픽
        self.pub_bbox = self.create_publisher(Float32MultiArray, '/target/bounding_box_x', 10)
        
        ### 퍼포먼스 확인을 위한 결과 이미지 발행 (옵션)
        # self.pub_result_img = self.create_publisher(CompressedImage, '/cam_rear/yolo/compressed', 10)
        
        ### YOLOv8 Nano 모델 로드 (가장 가벼운 모델)
        self.yolo = YOLO('yolov8n.pt')

        ### 기능 분리한 hsv 클래스 객체 생성
        self.tracker = HsvTracker()

        ### 프레임 제어용 변수 3프레임마다 yolo 적용
        self.frame_count = 0
        self.yolo_interval = 3
        
        ### 쉬는 프레임에서 재사용할 '이전 상태' 저장용 (ros2 토픽용 메세지 타입, data 미리 담아두기)
        self.last_bbox_msg = Float32MultiArray()
        self.last_bbox_msg.data = [-1.0, -1.0, -1.0]
        self.last_annotated_frame = None

    def image_callback(self, msg):
        ### 압축 이미지 해제
        self.frame_count += 1
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        ### YOLO 3프레임마다 한번씩 (classes=[0]으로 사람만 탐지, verbose=False로 터미널 로그 최소화)
        if self.frame_count % self.yolo_interval == 0:
            results = self.yolo(frame, classes=[0], verbose=False, device = 'cpu')
            bboxes = results[0].boxes.xyxy.cpu().numpy()
            
            if len(bboxes) > 0:
                target_bbox = self.tracker.get_target_bbox(frame, bboxes)

                if target_bbox is not None:
                    xmin, ymin, xmax, ymax = map(float, target_bbox)
                    center_x = (xmin + xmax) / 2.0
                    
                    # 퍼블리시용 데이터 갱신
                    self.last_bbox_msg.data = [xmin, center_x, xmax]
                
                ### 새로운 데이터를 구했으니 갱신
                self.last_bbox_msg.data = [xmin, center_x, xmax]
                # self.get_logger().info(f"현재 센터 값 x: {center_x:.1f}")

            else:
                self.last_bbox_msg.data = [-1.0, -1.0, -1.0]
            
            # ### 시각화 이미지 갱신 -> 프레임 너무 잡아먹음
            # self.last_annotated_frame = results[0].plot()

        ### 데이터 송출 (매 프레임 실행)
        self.pub_bbox.publish(self.last_bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()