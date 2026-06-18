import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateThroughPoses

class WaypointManager(Node):
    def __init__(self):
        # 1. 노드 이름 초기화
        super().__init__('waypoint_manager')
        
        # 2. 퍼블리셔, 서브스크라이버, 액션 클라이언트 선언 등
        self.get_logger().info("웨이포인트 매니저 노드가 시작되었습니다!")
        self.subscription = self.create_subscription(PoseArray, '/goal_x_y', self.sendgoal_cb, 10)
        # self.action_client = ActionClient(...)
        
        # 3. 내부 변수 (목적지 리스트 등) 초기화
        self.waypoint_list = []

    # 4. 콜백 함수나 실제 동작할 메서드들 작성
    def sendgoal_cb(self, msg):
        print("입력받은 x, y 좌표: ", msg.x, msg.y)

def main(args=None):
    # 1. ROS2 통신 초기화
    rclpy.init(args=args)

    # 2. 노드 객체 생성
    waypoint_manager = WaypointManager()

    try:
        # 3. 노드 실행 (콜백 함수들이 무한 루프로 대기)
        rclpy.spin(waypoint_manager)
    except KeyboardInterrupt:
        # Ctrl+C 등으로 종료 요청이 들어왔을 때의 예외 처리
        waypoint_manager.get_logger().info("사용자에 의해 노드가 종료됩니다.")
    finally:
        # 4. 자원 해제 및 ROS2 종료
        waypoint_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()