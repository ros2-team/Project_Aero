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

            # [FACTS AREA WRITE] QR 데이터 수신 시 백업 변수에 안전하게 적재
            if action_type == "qr_call_navigation" and new_route:
                # 🚨 기존 경유지(web_route_list)를 건드리지 않고, QR용 독립 변수에 우선 저장합니다.
                self.blackboard.qr_route_backup = new_route
                self.get_logger().info("📱 [QR 데이터 수신] 대기열 임시 적재 완료 (주행 완료 후 처리 예정)")

        except Exception as e:
            self.get_logger().error(f" ❌ QR 데이터 수신 중 오류 발생: {e}")