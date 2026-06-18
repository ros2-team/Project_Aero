import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publihser')

        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        time.sleep(1.0)
        self.send_goal()

        
    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = 1.0
        msg.pose.position.y = 1.0
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()