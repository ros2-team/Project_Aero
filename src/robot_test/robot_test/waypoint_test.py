import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import FollowWaypoints

class ActionManagerNode(Node):
    def __init__(self):
        super().__init__('action_manager_node')
        
        # 1. 입력 노드에서 쏜 토픽 구독
        self.subscription = self.create_subscription(
            PoseArray,
            '/goal_x_y',
            self.waypoints_callback,
            10)
            
        # 2. 액션 클라이언트 생성
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.get_logger().info('액션 매니저 대기 중... 좌표 토픽을 기다립니다.')

    def waypoints_callback(self, msg):
        self.get_logger().info(f'🎯 토픽 수신 완료! 총 {len(msg.poses)}개의 좌표로 액션을 쏩니다.')
        self.send_action_goal(msg.poses)

    def send_action_goal(self, poses):
        self.get_logger().info('액션 서버 연결 확인 중...')
        self._action_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        
        # Pose[] 배열을 액션이 요구하는 PoseStamped[] 형태로 예쁘게 포장하기
        waypoints_list = []
        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            waypoints_list.append(pose_stamped)

        goal_msg.poses = waypoints_list

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 서버가 Goal을 거절했습니다.')
            return

        self.get_logger().info('✅ Goal 수락됨! 주행 시작!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # 현재 향하고 있는 목적지 인덱스 실시간 출력
        current_idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'   -> [피드백] 현재 {current_idx}번째 목적지로 이동 중...')

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # STATUS_SUCCEEDED
            self.get_logger().info('🎉 모든 목적지 주행을 무사히 마쳤습니다!')
        else:
            self.get_logger().info(f'⚠️ 주행 중 문제가 발생했습니다. 상태 코드: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = ActionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()