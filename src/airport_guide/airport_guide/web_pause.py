#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class WebPauseNode(Node):
    def __init__(self, blackboard):
        super().__init__('web_pause_node')
        self.blackboard = blackboard

        # 웹브릿지(web_data.py)가 발행하는 제어 명령 토픽 구독
        self.command_sub = self.create_subscription(
            String,
            '/web/command',
            self.web_command_callback,
            10
        )
        self.get_logger().info("✅ [Data Layer] Web Pause 전처리 노드가 가동되었습니다.")

    def web_command_callback(self, msg):
        try:
            # 1) 웹 브릿지로부터 들어온 RAW JSON 파싱
            raw_data = json.loads(msg.data)
            action_type = raw_data.get("action")  # 웹에서 보내주는 명령 프로퍼티(type) 추출

            # 2) [FACTS AREA WRITE] 명령 종류별 블랙보드 상태 전처리 및 직접 대입
            if action_type == "pause_navigation":
                print("******************토픽 수신 완완완완완완*************************")
                # [일시정지] 메모리는 유지하되, 트리 진입을 위해 플래그 설정
                
                self.blackboard.is_paused = True 
                self.get_logger().info(" 일시정지 -> blackboard.is_paused = True")

            elif action_type == "resume_navigation":
                # [재개] 일시정지 플래그를 해제하여 하위 주행 브랜치로 제어권 복귀 유도
                self.blackboard.is_paused = False
                self.get_logger().info(" 주행 재개 -> blackboard.is_paused = False")

            elif action_type == "stop_navigation":
                # [주행 최종 종료/취소] 목적지를 비우고 초기 상태로 되돌리기 위한 전처리
                self.blackboard.is_paused = True  # 우선 로봇을 멈추도록 유도
                self.blackboard.goal_name = ""    # 임무 완전 삭제 (Reset)
                self.get_logger().warn(" 주행 최종 종료  -> 목적지 삭제 ")

        except Exception as e:
            self.get_logger().error(f" 웹 제어 명령 데이터 전처리 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    from airport_guide.blackboard import Blackboard
    db = Blackboard()
    node = WebPauseNode(blackboard=db)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()