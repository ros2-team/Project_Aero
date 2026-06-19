from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryNode(Node):

    def __init__(self, blackboard):

        super().__init__('battery_node')

        self.blackboard = blackboard

        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):

        self.blackboard.battery_level = msg.percentage   # 배터리 ㅍ센트 블랙보드에 저장  

        self.get_logger().info(
            f"Battery = {msg.percentage:.2f}"
        )
