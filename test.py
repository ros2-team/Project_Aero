#!/home/yeojin/yolo_test/venv/bin/python3

import cv2
import numpy as np
import time
from ultralytics import YOLO 
import rclpy                 # [ROS2 추가]
from rclpy.node import Node  # [ROS2 추가]
from std_msgs.msg import Int32MultiArray # [ROS2 추가] 데이터 배열 타입
from sensor_msgs.msg import CompressedImage

class YoloRosPublisher(Node):
    def __init__(self):
        super().__init__('yolo_ros_publisher')
        # '/yolo_counts'라는 이름으로 정수 배열 토픽을 발행할 준비를 합니다.
        self.subscriber = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cam_cb, 10)
        self.publisher_ = self.create_publisher(Int32MultiArray, '/yolo_counts', 10)
        self.model = YOLO('yolov8n.pt')
        
        # COCO 데이터셋 기준 클래스 번호 명시 사람 0, 차 2
        self.PERSON_CLASS_ID = 0
        self.CAR_CLASS_ID = 2

    def process_and_publish(self, frame):
        # YOLO 추론 수행
        results = self.model(frame, stream=True, device = 'cpu', imgsz = 320)
        
        person_count = 0
        car_count = 0
        annotated_frame = frame

        for r in results:
            annotated_frame = r.plot() # 바운딩 박스가 그려진 이미지 획득
            
            # 검출된 객체들의 클래스 ID를 분석하여 카운트 계산
            if r.boxes is not None:
                classes = r.boxes.cls.int().tolist()
                person_count = classes.count(self.PERSON_CLASS_ID)
                car_count = classes.count(self.CAR_CLASS_ID)

        # ROS2 메시지 객체 생성 및 데이터 매핑
        msg = Int32MultiArray()
        msg.data = [person_count, car_count]

        print("사람 수, 차량 수 : ", person_count, car_count)
        
        # 토픽 발행
        self.publisher_.publish(msg)
        
        return annotated_frame

    def cam_cb(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)

        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            self.get_logger().error('이미지 디코딩 실패')
            return
        
        display_frame = self.process_and_publish(cv_image)

        cv2.imshow('Webcam Test', display_frame)
        
        cv2.waitKey(1)

def main():
    # ROS2 인터페이스 초기화
    rclpy.init()
    yolo_node = YoloRosPublisher()

    # cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # if not cap.isOpened():
    #     print("Error: 가메라를 열 수 없습니다.")
    #     return

    print("웹캠 및 YOLOv8 ROS2 퍼블리셔 노드가 준비되었습니다. 'q'를 누르면 종료합니다.")

    try:
        while True:
            # ROS2 노드 내부 로직을 거쳐 카운트 토픽 발행 및 이미지 획득

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            # ROS2 컨텍스트 유지 (비동기 이벤트 처리용)
            rclpy.spin_once(yolo_node, timeout_sec=0.001)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()