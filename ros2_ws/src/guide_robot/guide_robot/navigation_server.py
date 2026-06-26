import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose


class NavigationServer(Node):

    def __init__(self, x, y, yaw):
        super().__init__("navigation_server")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.send_goal(x, y, yaw)

    def send_goal(self, x, y, yaw):

        self.get_logger().info(
            "Waiting for NavigateToPose action server..."
        )

        self.client.wait_for_server()

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # yaw 값을 quaternion으로 변환
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Send Goal x={x}, y={y}, yaw={yaw}"
        )

        send_goal_future = self.client.send_goal_async(goal)
        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result
        status = future.result().status

        self.get_logger().info(
            f"Navigation finished. status={status}"
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    if len(sys.argv) < 4:
        print(
            "Usage: ros2 run guide_robot navigation_server <x> <y> <yaw>"
        )
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])

    node = NavigationServer(x, y, yaw)

    rclpy.spin(node)

    node.destroy_node()


if __name__ == "__main__":
    main()