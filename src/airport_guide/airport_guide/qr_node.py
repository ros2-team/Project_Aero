#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from airport_guide.blackboard import Blackboard

class QrCallNode(Node):
    def __init__(self, blackboard):
        super().__init__('qr_call_node')
        self.blackboard = blackboard

        self.command_sub = self.create_subscription(
            String,
            '/web/command',
            self.web_command_callback,
            10
        )
        self.get_logger().info("✅ [Data Layer] QR Call 데이터 수신 노드가 가동되었습니다.")

    def web_command_callback(self, msg):
        try:
            raw_data = json.loads(msg.data)
            action_type = raw_data.get("action")
            payload = raw_data.get("payload", {})
            new_route = payload.get("route", [])

            # 🛠️ [수정] Flask의 send_route_to("qrcall", ...) 규격과 매칭
            if (action_type == "qrcall" or action_type == "qr_call_navigation") and new_route:
                # 안전한 디버깅 로그 출력
                self.get_logger().info(f"!!!!!qr 위치 수신 완료: {new_route[0].get('location_name', '알 수 없음')}!!!!!!!")
                
                # 기존 경유지를 건드리지 않고 백업 변수에 적재
                self.blackboard.qr_route_backup = new_route
                self.get_logger().info("[QR 데이터] 대기열 임시 적재 완료 (공백 상태)")

        except Exception as e:
            self.get_logger().error(f" QR 데이터 수신 중 오류 발생: {e}")