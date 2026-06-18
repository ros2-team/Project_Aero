import json
import time
import math 
import threading 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry    
import paho.mqtt.client as mqtt

class WebToTurtlebotGateway(Node):
    def __init__(self):
        super().__init__('web_turtlebot_gateway')

        # 1. 터틀봇 모터 제어용 ROS2 Publisher
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # 로봇의 실제 위치(Odometry) 구독자
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # 로봇 상태 및 목적지 관리 변수
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.current_robot_yaw = 0.0  
        self.remaining_path = []
        self.is_moving = False  

        # 2. MQTT 클라이언트 설정
        try:
            self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            self.mqtt_client = mqtt.Client() 
        
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        try:
            self.mqtt_client.connect("127.0.0.1", 1883, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("🚀 MQTT 루프 시작 성공")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 브로커 연결 실패: {e}")

        self.get_logger().info("터틀봇 전/후진 완전 자율 조향 노드가 시작되었습니다.")

    def on_connect(self, client, userdata, flags, rc, *args):
        if rc == 0:
            self.get_logger().info("✔ MQTT 브로커 연결 완료")
            client.subscribe("robot/navigation/path")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_robot_x = msg.pose.pose.position.x
        self.current_robot_y = msg.pose.pose.position.y
        
        quaternion = msg.pose.pose.orientation
        self.current_robot_yaw = self.get_yaw_from_quaternion(quaternion)

        if len(self.remaining_path) > 0 and self.is_moving:
            target = self.remaining_path[0]
            coord = target.get('coordinate', {})
            target_x = coord.get('x', 0.0)
            target_y = coord.get('y', 0.0)

            distance = math.sqrt((target_x - self.current_robot_x)**2 + (target_y - self.current_robot_y)**2)

            # 도착 판정 기준 (0.15m 이내 진입 시 멈춤)
            if distance < 3.0:
                self.is_moving = False 
                self.get_logger().info(f"[목적지 최종 도달] {target['name']}에 정확히 도착했습니다.")

                stop_cmd = Twist()
                self.velocity_publisher.publish(stop_cmd)

                status_payload = [{"instanceId": p.get("instanceId"), "name": p["name"], "status": "pending"} for p in self.remaining_path]
                status_payload[0]["status"] = "finish"

                self.remaining_path.pop(0)

                json_status = json.dumps(status_payload, ensure_ascii=False)
                self.mqtt_client.publish("robot/navigation/status", json_status, qos=1)

    def on_message(self, client, userdata, msg):
        try:
            self.remaining_path = json.loads(msg.payload.decode())
            if len(self.remaining_path) > 0:
                t = threading.Thread(target=self.control_robot_and_respond)
                t.start()
        except Exception as e:
            self.get_logger().error(f"데이터 수신 에러: {e}")

    # 💡 [핵심 수정] 전진 및 후진 주행 상황을 판단하는 자율 조향 루프
    def control_robot_and_respond(self):
        try:
            target = self.remaining_path[0]
            coord = target.get('coordinate', {})
            target_x = coord.get('x', 0.0)
            target_y = coord.get('y', 0.0)
            
            self.get_logger().info(f"[목적지 추적 주행 시작] {target['name']} (X: {target_x}, Y: {target_y})")
            self.is_moving = True

            cmd = Twist()
            
            while self.is_moving and rclpy.ok():
                inc_x = target_x - self.current_robot_x
                inc_y = target_y - self.current_robot_y
                
                angle_to_target = math.atan2(inc_y, inc_x)
                angle_error = angle_to_target - self.current_robot_yaw
                angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

                # 💡 [후진 판정 조건] 각도 오차가 90도(1.57 라디안)보다 크면 목적지가 뒤에 있음
                if abs(angle_error) > 1.57:
                    # 후진 제어 모드 적용
                    cmd.linear.x = -0.15  # 음수 입력으로 역방향 구동
                    
                    # 후진할 때 조향 각도 오차 역계산 연산 (-PI ~ +PI 스케일 정규화)
                    backward_error = angle_error - math.copysign(math.pi, angle_error)
                    cmd.angular.z = 2.0 * backward_error  # 뒤로 가면서 미세 조향
                else:
                    # 전진 제어 모드 적용
                    if abs(angle_error) > 0.6:
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.5 if angle_error > 0 else -0.5
                    else:
                        cmd.linear.x = 0.15  # 양수 입력으로 순방향 구동
                        cmd.angular.z = 2.0 * angle_error  

                self.velocity_publisher.publish(cmd)
                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"주행 알고리즘 스레드 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebToTurtlebotGateway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()