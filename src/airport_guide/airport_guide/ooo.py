#!/usr/bin/env python3
import time
import requests
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WebBridgeNode(Node):
    def __init__(self, blackboard):
        super().__init__("web_bridge_node")
        self.blackboard = blackboard
        self.flask_base_url = "http://192.168.0.9:5000"
        self.last_handled_command_id = -1
        self.polling_interval = 0.5

        # 내비게이션 래치 상태 저장 레이어 보완
        # 구조: (web_status, web_display_index, is_paused)
        self.last_nav_status = None
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)

        # 주행 중인 보정 경로 데이터 로컬 캐시
        self.active_processed_route = []

        # 행동트리 상태 모니터링 구독
        self.bt_status_sub = self.create_subscription(
            String, "/robot/bt_status", self._bt_status_callback, 10
        )
        self.get_logger().info("확정 API 기반 ROS2 Web Bridge Node 가동 시작.")
        self.polling_thread = threading.Thread(target=self._command_polling_loop, daemon=True)
        self.polling_thread.start()
        
    def _command_polling_loop(self):
        while rclpy.ok():
            try:
                response = requests.get(f"{self.flask_base_url}/api/robot/command", timeout=1.0)
                if response.status_code == 200:
                    res_data = response.json()
                    if res_data.get("status") == "success":                     
                        command_data = res_data.get("command", {})
                        has_command = command_data.get("has_command", False)
                        is_handled = command_data.get("is_handled", True)                       
                        command_id = command_data.get("command_id", -1)

                        if has_command and not is_handled and (command_id != self.last_handled_command_id):
                            self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_data.get('type')}")
                            
                            raw_route = command_data.get("route", [])
                            processed_route = []
                            current_location = command_data.get("location_name", "대기소") 

                            if raw_route:
                                first_wp = raw_route[0]
                                first_name = first_wp.get("location_name")
                                first_order = first_wp.get("order", 0)

                                if (current_location == "게이트 A" and first_name == "게이트 B") or \
                                   (current_location == "게이트 B" and first_name == "게이트 A"):
                                    self.get_logger().info(f"🚀 [Route Planner] 직선구간 프리패스! (선행 경유지 생략)")
                                elif first_name in ["게이트 A", "게이트 B", "게이트 C"]:
                                    initial_mid = {
                                        "location_code": "MID_RIGHT",
                                        "location_name": "Corner_Right_Mid",
                                        "order": first_order, 
                                        "x": 0.44, "y": 0.08, "yaw": 0.0,
                                        "is_mid_point": True
                                    }
                                    processed_route.append(initial_mid)
                                else:
                                    initial_mid = {
                                        "location_code": "MID_LEFT",
                                        "location_name": "Corner_Left_Mid",
                                        "order": first_order,
                                        "x": -0.37, "y": -0.08, "yaw": 0.0,
                                        "is_mid_point": True
                                    }
                                    processed_route.append(initial_mid)

                            for i in range(len(raw_route)):
                                processed_route.append(raw_route[i]) 
                                if i < len(raw_route) - 1:
                                    current_name = raw_route[i].get("location_name")
                                    next_name = raw_route[i+1].get("location_name")
                                    current_order = raw_route[i].get("order", 0)
                                    
                                    if (current_name == "게이트 A" and next_name == "게이트 C") or \
                                       (current_name == "게이트 C" and next_name == "게이트 A") or \
                                       (current_name == "게이트 B" and next_name == "게이트 C") or \
                                       (current_name == "게이트 C" and next_name == "게이트 B"):
                                        right_mid = {
                                            "location_code": "MID_RIGHT",   
                                            "location_name": "Corner_Right_Mid",
                                            "order": current_order,          
                                            "x": 0.44, "y": 0.08, "yaw": 0.0,                     
                                            "is_mid_point": True
                                        }
                                        processed_route.append(right_mid)
                                    elif (current_name == "게이트 A" and next_name == "게이트 B") or \
                                         (current_name == "게이트 B" and next_name == "게이트 A"):
                                        pass 
                                    else:
                                        left_mid = {
                                            "location_code": "MID_LEFT",
                                            "location_name": "Corner_Left_Mid",
                                            "order": current_order,
                                            "x": -0.37, "y": -0.08, "yaw": 0.0,
                                            "is_mid_point": True
                                        }
                                        processed_route.append(left_mid)

                            self.active_processed_route = processed_route
                            command_data["route"] = processed_route
                            
                            self._publish_to_behavior_tree(command_data)
                            self._mark_command_as_handled(command_id)
                            self.last_handled_command_id = command_id
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Flask 서버 폴링 중 통신 실패: {e}")
            time.sleep(self.polling_interval)

    def _publish_to_behavior_tree(self, command_data: dict):
        msg = String()
        msg.data = json.dumps({
            "action": command_data.get("type"),
            "command_id": command_data.get("command_id"),
            "payload": {"route": command_data.get("route", [])}
        }, ensure_ascii=False)
        self.web_command_pub.publish(msg)

    def _mark_command_as_handled(self, command_id: int):
        try:
            url = f"{self.flask_base_url}/api/robot/command/handled"
            headers = {"Content-Type": "application/json"}
            payload = {"command_id": command_id}
            requests.post(url, data=json.dumps(payload), headers=headers, timeout=1.0)
        except Exception as e:
            self.get_logger().error(f"처리 완료 보고 중 예외 발생: {e}")

    def _convert_bt_goal_state_to_web_status(self, bt_data: dict) -> str:
        if bt_data.get("navigation_finished", False):
            return "finished"
        if bt_data.get("is_paused", False):
            return "paused"

        goal_state = bt_data.get("goal_state", bt_data.get("status", "idle"))
        goal_state = str(goal_state).lower()
        current_index = int(bt_data.get("current_index", 0))

        route = bt_data.get("route", [])
        route_length = len(route) if isinstance(route, list) else 0

        if goal_state == "idle": return "idle"
        if goal_state in ["sent", "running"]: return "moving"
        if goal_state == "canceling": return "stopped"
        if goal_state == "done":
            if route_length > 0 and current_index >= route_length:
                return "finished"
            return "moving"
        if goal_state in ["moving", "paused", "stopped", "finished"]:
            return goal_state
        return "idle"

    def _bt_status_callback(self, msg: String):
        try:
            bt_data = json.loads(msg.data)
            headers = {"Content-Type": "application/json"}

            if "robot_status" in bt_data:
                url = f"{self.flask_base_url}/api/robot/status"
                requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
                return

            elif "status" in bt_data or "goal_state" in bt_data:
                current_index = int(bt_data.get("current_index", 0))
                goal_state = str(bt_data.get("goal_state", bt_data.get("status", ""))).lower()

                is_mid_point_active = False
                if current_index < len(self.active_processed_route):
                    if self.active_processed_route[current_index].get("is_mid_point", False):
                        is_mid_point_active = True

                if goal_state == "done" and current_index > 0:
                    prev_index = current_index - 1
                    if prev_index < len(self.active_processed_route):
                        if self.active_processed_route[prev_index].get("is_mid_point", False):
                            is_mid_point_active = True

                web_status = self._convert_bt_goal_state_to_web_status(bt_data)

                if is_mid_point_active:
                    web_status = "moving"

                web_display_index = 0
                for i in range(current_index):
                    if i < len(self.active_processed_route):
                        if not self.active_processed_route[i].get("is_mid_point", False):
                            web_display_index += 1
                            
                if web_status == "finished":
                    total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
                    web_display_index = total_real_dest

                navigation_payload = {
                    "status": web_status,
                    "current_index": web_display_index
                }

                # ================================================================
                # 🎯 [스마트 상태 변경 판정 필터 메커니즘]
                # ================================================================
                should_send = False
                trigger_reason = ""

                if self.last_nav_status is None:
                    # 최초 데이터는 무조건 동기화
                    should_send = True
                    trigger_reason = "최초 런타임 동기화"
                else:
                    # 직전에 '진짜 Flask로 쐈던' 데이터들을 복원
                    last_web_status = self.last_nav_status[0]
                    last_web_index = self.last_nav_status[1]
                    last_web_paused = self.last_nav_status[2]

                    current_paused = bt_data.get("is_paused", False)

                    # 핵심 제어 변수 3개 중 하나라도 '실질적인 변화'가 있을 때만 트리거
                    if (last_web_status != navigation_payload["status"] or 
                        last_web_index != navigation_payload["current_index"] or 
                        last_web_paused != current_paused):
                        
                        should_send = True
                        trigger_reason = f"상태 전이 발생 -> Status:({last_web_status}->{navigation_payload['status']}), Index:({last_web_index}->{navigation_payload['current_index']}), Paused:({last_web_paused}->{current_paused})"

                # 트래픽 전송 및 래치 메모리 최신화 (중복 블록 제거 및 정상 단일화)
                if should_send:
                    self.get_logger().info(f"📢 [Flask 전송 확정] 사유: {trigger_reason} -> status: {navigation_payload['status']}, index: {navigation_payload['current_index']}")
                    
                    try:
                        url = f"{self.flask_base_url}/api/robot/navigation"
                        res = requests.post(url, data=json.dumps(navigation_payload), headers=headers, timeout=1.0)
                        
                        if res.status_code == 200:
                            # 전송이 성공한 시점에만 로컬 저장소(last_nav_status)를 캐시 잠금(Latching)합니다.
                            self.last_nav_status = (
                                navigation_payload["status"],
                                navigation_payload["current_index"],
                                bt_data.get("is_paused", False)
                            )
                        else:
                            self.get_logger().error(f"Flask 전송 실패 (HTTP Code: {res.status_code})")
                    except Exception as e:
                        self.get_logger().error(f"Flask 통신 예외 발생: {e}")
                else:
                    # 필터링되어 드롭된 불필요 트래픽 로그 (디버깅용)
                    pass

        except Exception as e:
            self.get_logger().error(f"navigation_state 파싱 및 필터링 실패: {e}")