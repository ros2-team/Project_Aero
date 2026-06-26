import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose


class NavigationServer(Node):

    def __init__(self, route):
        super().__init__("navigation_server")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.route = route
        self.current_index = 0

        self.get_logger().info(
            f"Navigation Server Started. goals={len(self.route)}"
        )

        self.send_next_goal()

    def send_next_goal(self):

        if self.current_index >= len(self.route):
            self.get_logger().info("All goals completed")
            rclpy.shutdown()
            return

        target = self.route[self.current_index]

        x = target["x"]
        y = target["y"]
        yaw = target["yaw"]

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

        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Send Goal {self.current_index + 1}/{len(self.route)} "
            f"x={x}, y={y}, yaw={yaw}"
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

        status = future.result().status

        self.get_logger().info(
            f"Goal finished. status={status}"
        )

        self.current_index += 1

        self.send_next_goal()


def parse_route_from_args(args):

    if len(args) < 3:
        raise ValueError(
            "Usage: ros2 run guide_robot navigation_server <x> <y> <yaw> [<x> <y> <yaw> ...]"
        )

    if len(args) % 3 != 0:
        raise ValueError(
            "Arguments must be groups of 3: x y yaw"
        )

    route = []

    for i in range(0, len(args), 3):
        route.append({
            "x": float(args[i]),
            "y": float(args[i + 1]),
            "yaw": float(args[i + 2])
        })

    return route


def main(args=None):

    rclpy.init(args=args)

    try:
        route = parse_route_from_args(sys.argv[1:])
    except ValueError as e:
        print(e)
        rclpy.shutdown()
        return

    node = NavigationServer(route)

    rclpy.spin(node)

    node.destroy_node()


if __name__ == "__main__":
    main()