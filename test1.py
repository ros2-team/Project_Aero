#!/home/yeojin/yolo_test/venv/bin/python3

import cv2
import time
from ultralytics import YOLO 
import rclpy                 
from rclpy.node import Node  
from std_msgs.msg import String 

class YoloRosPublisher(Node):
    def __init__(self):
        super().__init__('yolo_ros_publisher')
        # [핵심 수정] 보내는 토픽의 규격을 Int32MultiArray에서 String으로 완전히 변경합니다.
        self.publisher_ = self.create_publisher(String, '/yolo_counts', 10) # 데이터 타입, 채널이름, 10개만 
        self.model = YOLO('yolov8n.pt')

    def process_and_publish(self, frame):  
        #(연산 및 송신 파트): OpenCV가 뜯어온 카메라 프레임을 YOLO AI에 입력하여 사물 이름과 중심 좌표를 추출
        #추출된 텍스트들을 묶어 msg.data에 담은 후, 등록한 채널로 밀어냄


        # 1. YOLOv8 모델로 실시간 추론 수행
        # results 안에 데이터 패키지가 있음 (ex_cls, conf, boxes)
        results = self.model(frame, stream=True)
        
        annotated_frame = frame  # 원본카메라 화면을 넣음 
        log_list = []
        # 이번 프레임에서 발견된 사물들의 실시간 개수를 세기 위한 사전
        detected_counts = {}
        

        for r in results:
            annotated_frame = r.plot() # 바운딩 박스 시각화 함수 
            
            if r.boxes is not None:
                for box in r.boxes:
                    # 발견된 객체의 클래스 고유 번호 추출
                    cls_id = int(box.cls[0])
                    # 번호를 실제 사물 이름으로 변환
                    cls_name = self.model.names[cls_id]
        
                    # 발견된 사물의 등장 횟수를 동적으로 계산 (1씩 누적)
                    # person 1,2,3,4...를 만듦
                    detected_counts[cls_name] = detected_counts.get(cls_name, 0) + 1
                    current_idx = detected_counts[cls_name]
                    
                    # 중심점 좌표 데이터 추출 및 정수 변환
                    xywh = box.xywh[0].int().tolist()
                    center_x = xywh[0]
                    center_y = xywh[1]

                    # 이름, 고유 순번, 좌표를 매칭하여 리스트에 추가
                    log_list.append(f"{cls_name}{current_idx}[x:{center_x}, y:{center_y}]")

        # -----------------------------------------------------------------
        # [터미널 출력 및 ROS2 전송]
        # -----------------------------------------------------------------
        if log_list:
            log_output = "\n".join(log_list)
            print(log_output)
            
            # 수신 노드로 문자열 토픽 전송 (여기도 String 객체)
            msg = String()
            msg.data = log_output
            self.publisher_.publish(msg)
        # -----------------------------------------------------------------
        
        return annotated_frame

def main():
    rclpy.init()
    yolo_node = YoloRosPublisher()

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Error: 카메라를 열 수 없습니다.")
        return

    print("웹캠 및 YOLOv8 ROS2 퍼블리셔 노드가 준비되었습니다. 'q'를 누르면 종료합니다.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            display_frame = yolo_node.process_and_publish(frame)
            cv2.imshow('Webcam Test', display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # time.sleep(1.0) # 프레임 주기 조절 
                
            rclpy.spin_once(yolo_node, timeout_sec=0.001)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()