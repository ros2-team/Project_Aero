#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from behavior_tree.blackboard import Blackboard

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

    def web_command_callback(self, msg):
        try:
            raw_data = json.loads(msg.data)
            action_type = raw_data.get("action")
            payload = raw_data.get("payload", {})
            new_route = payload.get("route", [])

            # Flask의 send_route_to("qrcall", ...) 규격과 매칭
            if (action_type == "qrcall" or action_type == "qr_call_navigation") and new_route:
                # 기존 경유지를 건드리지 않고 qr_route_backup에 적재
                self.blackboard.qr_route_backup = new_route

        except Exception as e:
            self.get_logger().error(f" QR 데이터 수신 중 오류 발생: {e}")