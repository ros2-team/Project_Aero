#!/home/yeojin/yolo_test/venv/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from my_robot_msgs.msg import Detection  
from cv_bridge import CvBridge
# from ultralytics import YOLO
import cv2

class YoloRosPublisher(Node):
    def __init__(self):
        super().__init__('yolo_ros_publisher')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # 1단계 원시 토픽 발행 (가공 노드인 ga_sub_node가 구독함)
        # self.obj_pub = self.create_publisher(Detection, '/yolo_raw_detection', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.frame_count = 0
        self.get_logger().info('YOLO Raw Publisher Started.')

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 10 != 0: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        results = self.model(frame, verbose=False)
        for r in results:
            if r.boxes is None: continue
            for box in r.boxes:
                if float(box.conf.item()) < 0.2: continue

                cls_id = int(box.cls.item())
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                print("YOLO 검출:", cls_name)

                # pub_msg = Detection()

                # pub_msg.class_name = cls_name
                # pub_msg.direction = ""      # 아직 방향 계산 전
                # pub_msg.distance = 0.0      # 아직 거리 계산 전

                # pub_msg.x = (x1 + x2) // 2
                # pub_msg.y = (y1 + y2) // 2

                # print("보내기 전 =", pub_msg.class_name)

                # self.obj_pub.publish(pub_msg)

        annotated_frame = r.plot()
        cv2.imshow("YOLO View", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()