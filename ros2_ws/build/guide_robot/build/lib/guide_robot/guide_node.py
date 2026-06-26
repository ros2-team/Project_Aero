import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GuideNode(Node):
    def __init__(self):
        super().__init__("guide_node")
        
        self.subsrciption = self.create_subscription(
            String,
            "/robot_call",
            self.call_callback,
            10
        )
        self.get_logger().info(
            "Guide Node Start"
        )

    def call_callback(self,msg):
        location_code = msg.data
        self.get_logger().info(
            f"Call Receive : {location_code}"
        )
    
    def main(args=None):
        rclpy.init(args=args)
        node = GuideNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    
    if __name__ == "__main__":
        main()