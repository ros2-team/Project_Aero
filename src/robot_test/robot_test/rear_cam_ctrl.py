import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # 인지 노드(Perception)에서 쏴주는 타겟 데이터 구독
        self.target_sub = self.create_subscription(Float32MultiArray, '/target_data', self.target_callback, 10)
        self.get_logger().info("✅ Control Node Started. Waiting for /target_data...")

    def target_callback(self, msg):
        # 배열 길이 예외 방어막 (인덱스 에러 방지)
        if len(msg.data) < 3:
            self.get_logger().error("데이터 못 받음 ")
            return

        # 데이터 파싱
        target_id = msg.data[0]
        center_x = msg.data[1]
        area = msg.data[2]

        # 🛑 [예외 처리] 타겟을 놓쳤을 때 (-1.0, 0.0, 0.0)
        if target_id == -1.0:
            self.get_logger().warn(" 타겟 놓침! (ID: -1.0)")
            
            # TODO: 나중에 여기에 '30틱(프레임) 대기'하는 오클루전 방어 로직을 넣으면 됨.
            # 지금은 뼈대니까 'S'(정지) 명령을 내릴 준비만 한다고 생각하면 됨.
            return
        
        # 🟢 [정상 처리] 타겟을 안정적으로 물고 있을 때 데이터 출력
        self.get_logger().info(
            f"🎯 Tracking - ID: {int(target_id)} | X: {center_x:.1f} | Area: {area:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()