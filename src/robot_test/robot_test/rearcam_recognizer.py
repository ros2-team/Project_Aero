#!/home/yeojin/yolo_test/venv/bin/python3
import numpy as np
import time
from ultralytics import YOLO 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2
import sys

class RearCamera(Node):
    def __init__(self):
        super().__init__('rear_recognizer')
        # '/yolo_counts'라는 이름으로 정수 배열 토픽을 발행할 준비를 합니다.
        self.subscriber = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.cam_cb, 10)
        self.model = YOLO('yolov8n.pt')
        
        # COCO 데이터셋 기준 클래스 번호 명시 사람 0, 차 2
        self.PERSON_CLASS_ID = 0
        self.bridge = CvBridge()


    def cam_cb(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        if cv_image is None:
            self.get_logger().error('이미지 디코딩 실패')
            return
        
        display_frame, bbox = self.process_and_publish(cv_image)
        hist = self.cal_hsv(bbox)

        cv2.imshow('Webcam Test', display_frame)
        cv2.waitKey(1)

    def process_and_publish(self, frame):
        # YOLO 추론 수행
        results = self.model(frame, stream=True, device = 'cpu', imgsz = 320, verbose = False)
        person_count = 0
        annotated_frame = frame

        for r in results:
            annotated_frame = r.plot() # 바운딩 박스가 그려진 이미지 획득
            
            # 검출된 객체들의 클래스 ID를 분석하여 카운트 계산
            if r.boxes is not None:
                classes = r.boxes.cls.int().tolist()
                person_count = classes.count(self.PERSON_CLASS_ID)
                
                for box in r.boxes:
                    class_id = int(box.cls[0])
                    if class_id == self.PERSON_CLASS_ID:
                    # xyxy 좌표를 뽑아서 정수(int) 리스트로 변환!
                    # 결과: [x_min, y_min, x_max, y_max]
                        bbox = box.xyxy[0].int().tolist() 

        # ROS2 메시지 객체 생성 및 데이터 매핑
        # msg = Int32MultiArray()
        # msg.data = [person_count, car_count]

        print("사람 수 : ", person_count)
        
        # # 토픽 발행
        # self.publisher_.publish(msg)
        
        return annotated_frame, bbox

    def cal_hsv(self, data):
        
        pass


def main(args=None):
    # 1. ROS2 통신 초기화
    rclpy.init(args=args)

    # 2. 노드 객체 생성
    rc = RearCamera()

    try:
        # 3. 노드 실행 (콜백 함수들이 무한 루프로 대기)
        rclpy.spin(rc)

    except KeyboardInterrupt:
        # Ctrl+C 등으로 종료 요청이 들어왔을 때의 예외 처리
        rc.get_logger().info("사용자에 의해 노드가 종료됩니다.")

    finally:
        # 4. 자원 해제 및 ROS2 종료
        rc.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()