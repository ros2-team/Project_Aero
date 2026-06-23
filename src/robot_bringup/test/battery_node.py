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

        # self.get_logger().info(
        #     f"Battery = {msg.percentage:.2f}"
        # )

    # 장애물 또는 센서 인지 노드 내부 예시
    # def sensor_callback(self, msg):
    #     토픽이 들어올 때마다 블랙보드 시간을 지금 시간으로 계속 바꿔 써줌
    #     self.blackboard.last_sensor_time = time.time()
        
        # (옵션) 인지 노드 자체에서 통신 끊김을 감지했다면 플래그를 켜줌
        # self.blackboard.sensor_timeout = False

