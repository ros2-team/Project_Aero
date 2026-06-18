import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import threading
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateThroughPoses

class SendCustomGoal(Node):
    def __init__(self):
        # 노드 이름 초기화
        super().__init__('send_goal')
        self.publisher = self.create_publisher(PoseArray, '/goal_x_y', 10)

        self.input_thread = threading.Thread(target = self.get_user_input) #백그라운드로 해당 함수 선언
        self.input_thread.start() # 백그라운드 스레드에서 시작
        # 내부 변수 (목적지 리스트 등) 초기화
        self.waypoints = []

    # 4. 콜백 함수나 실제 동작할 메서드들 작성
    def get_user_input(self):
        self.get_logger().info('좌표를 입력해주세요. go 를 입력하면 로봇이 출발합니다 !!! ')
        while rclpy.ok(): # rclpy 상태 확인 rclpy.shutdown()시 false로 바뀜
            x = input()
            if x.strip().lower() == 'go':
                self.publish_waypoint()
                self.waypoints.clear()
            else:
                try:
                    point_x, point_y = x.split()
                    pose = Pose()
                    pose.position.x = float(point_x)
                    pose.position.y = float(point_y)
                    self.waypoints.append(pose)
                except ValueError:
                    self.get_logger().info('정확한 값을 입력해주세요 ^^')

    def publish_waypoint(self):
        if not self.waypoints:
            self.get_logger().info('좌표가 없습니다.')
            return
        
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.poses = self.waypoints

        self.publisher.publish(msg) 

def main(args=None):
    # 1. ROS2 통신 초기화
    rclpy.init(args=args)

    # 2. 노드 객체 생성
    send_goal = SendCustomGoal()

    try:
        # 3. 노드 실행 (콜백 함수들이 무한 루프로 대기)
        rclpy.spin(send_goal)
    except KeyboardInterrupt:
        # Ctrl+C 등으로 종료 요청이 들어왔을 때의 예외 처리
        send_goal.get_logger().info("사용자에 의해 노드가 종료됩니다.")
    finally:
        # 4. 자원 해제 및 ROS2 종료
        send_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()