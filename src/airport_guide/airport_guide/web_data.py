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

        # 🛠️ 수정한 규격에 맞게 내비게이션 상태만 변경 감지하기 위한 변수
        self.last_nav_status = None
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)

        # 행동트리 상태 모니터링
        self.bt_status_sub = self.create_subscription(
            String, "/robot/bt_status", self._bt_status_callback, 10
        )
        self.get_logger().info("확정 API 기반 ROS2 Web Bridge Node 가동 시작.")
        self.polling_thread = threading.Thread(target=self._command_polling_loop, daemon=True)
        self.polling_thread.start()
        
    def _command_polling_loop(self):
        """설정된 주기마다 Flask 서버를 폴링하며 새로운 명령이 생성되었는지 감시하는 루프"""
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

                        # 새로운 명령이 들어오고 새로들어온 명령어가 이전 id보다 크면 신규 명령으로 판단
                        if has_command and not is_handled and (command_id != self.last_handled_command_id):
                            self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_data.get('type')}")
                            self.get_logger().info(
                                "\n" + "="*60 +
                                f"\n📥 [WEB DATA RAW JSON] ID: {command_id}" +
                                f"\n🔹 전체 내용:\n{json.dumps(command_data, indent=2, ensure_ascii=False)}" +
                                "\n" + "="*60
                            )

                            ############## 경로 보정 ###############
                            raw_route = command_data.get("route", [])
                            processed_route = []

                            # 경로 리스트를 돌면서 중간 경유지 자동 주입
                            for i in range(len(raw_route)):
                                current_wp = raw_route[i]
                                processed_route.append(current_wp)  # 현재 위치 추가
                                
                                # 마지막 목적지가 아니라면, '현재 위치'와 '다음 위치' 사이의 이동을 검사
                                if i < len(raw_route) - 1:
                                    current_name = current_wp.get("location_name")
                                    next_name = raw_route[i+1].get("location_name")
                                    current_order = current_wp.get("order", 0)
                                    
                                    # 🚦 1. Gate A ↔ C 또는 Gate B ↔ C 구간인 경우 -> Right 경유지 주입
                                    if (current_name == "게이트 A" and next_name == "게이트 C") or \
                                       (current_name == "게이트 C" and next_name == "게이트 A") or \
                                       (current_name == "게이트 B" and next_name == "게이트 C") or \
                                       (current_name == "게이트 C" and next_name == "게이트 B"):
                                        
                                        right_mid = {
                                            "order": current_order,          # 에러 방지용 순서 동기화
                                            "location_code": "MID_RIGHT",   # 에러 방지용 더미 코드
                                            "location_name": "Corner_Right_Mid",
                                            "x": 0.44,
                                            "y": 0.08,
                                            "yaw": 0.0,                     # 에러 방지용 더미 방향
                                            "is_mid_point": True
                                        }
                                        processed_route.append(right_mid)
                                        self.get_logger().info(f"🔄 [Route Planner] {current_name} ↔ {next_name} (특수구간) -> Right 경유지 강제 주입")

                                    # 2.게이트 A ↔ 게이트 B 구간인 경우 -> 경유지 없이 프리패스!
                                    elif (current_name == "게이트 A" and next_name == "게이트 B") or \
                                        (current_name == "게이트 B" and next_name == "게이트 A"):
                                        
                                        self.get_logger().info(f"🚀 [Route Planner] {current_name} ↔ {next_name} (직선구간) -> 경유지 패스, 최단거리 직행!")
                                        pass # 아무것도 append 하지 않고 그냥 다음 목적으로 넘어갑니다.
                                                                # 🚦 2. 화장실, 면세점 등 그 외의 모든 구간 -> 무조건 Left 경유지 주입
                                    else:
                                        left_mid = {
                                            "order": current_order,
                                            "location_code": "MID_LEFT",
                                            "location_name": "Corner_Left_Mid",
                                            "x": -0.43,
                                            "y": 0.08,
                                            "yaw": 0.0,
                                            "is_mid_point": True
                                        }
                                        processed_route.append(left_mid)
                                        self.get_logger().info(f"🔄 [Route Planner] {current_name} ↔ {next_name} (일반구간) -> Left 경유지 무조건 주입")

                            # self.blackboard.web_action = command_data.get("type")
                            # self.blackboard.web_route_list = processed_route
                            # self.blackboard.web_last_update_time = time.time()
                            # self.get_logger().info("📊 Planner가 보정한 데이터를 Blackboard 변수에 직대입 동기화 완료.")
                            
                            # self.get_logger().info(
                            #     "\n" + "="*60 +
                            #     f"\n📥 [WEB DATA RAW JSON] ID: {command_id}" +
                            #     f"\n🔹 전체 내용:\n{json.dumps(command_data, indent=2, ensure_ascii=False)}" +
                            #     "\n" + "="*60
                            # )
                            
                            # raw_route = command_data.get("route", [])  # 웹에서 온 원래 경로 리스트
                            # processed_route = []                       # 새로 가공해서 담을 빈 리스트
                            
                            # for wp in raw_route:
                            #     processed_route.append(wp)
                            
                            command_data["route"] = processed_route
                            
                            self._publish_to_behavior_tree(command_data)
                            self._mark_command_as_handled(command_id)
                            self.last_handled_command_id = command_id
            
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Flask 서버 폴링 중 통신 실패: {e}")
            
            time.sleep(self.polling_interval)

    def _publish_to_behavior_tree(self, command_data: dict):
        """로봇 내부 행동트리가 수신할 수 있도록 ROS2 토픽으로 직렬화하여 발행"""
        msg = String()

        msg.data = json.dumps({
            "action": command_data.get("type"),
            "command_id": command_data.get("command_id"),
            "payload": {
                "route": command_data.get("route", [])
            }
        }, ensure_ascii=False)

        self.web_command_pub.publish(msg)
        
        self.get_logger().info("행동트리 수신용 ROS2 토픽 발행 완료.")

    def _mark_command_as_handled(self, command_id: int):
        try:
            url = f"{self.flask_base_url}/api/robot/command/handled"
            headers = {"Content-Type": "application/json"}
            payload = {"command_id": command_id}
            res = requests.post(url, data=json.dumps(payload), headers=headers, timeout=1.0)
            if res.status_code == 200:
                self.get_logger().info(f"Flask 서버에 명령 처리 완료 보고 성공 (ID: {command_id})")
            else:
                self.get_logger().error(f"처리 완료 보고 실패 (HTTP 상태 코드: {res.status_code})")
        except Exception as e:
            self.get_logger().error(f"처리 완료 보고 중 예외 발생: {e}")

    def _convert_bt_goal_state_to_web_status(self, bt_data: dict) -> str:
        
        # 행동트리 내부 상태를 웹 UI에서 사용하는 navigation status로 변환한다.

        # BT 내부 상태:
        # idle, sent, running, canceling, done

        # Web 상태:
        # idle, moving, paused, stopped, finished

        if bt_data.get("navigation_finished", False):
            return "finished"
        
        if bt_data.get("is_paused", False):
            return "paused"

        # 권장: 행동트리에서 goal_state로 보내는 경우
        goal_state = bt_data.get("goal_state")

        # 현재 코드 호환용: 행동트리에서 status로 보내는 경우도 처리
        if goal_state is None:
            goal_state = bt_data.get("status", "idle")

        goal_state = str(goal_state).lower()

        current_index = int(bt_data.get("current_index", 0))

        route_length = bt_data.get("route_length")

        if route_length is None:
            route = bt_data.get("route", [])
            if isinstance(route, list):
                route_length = len(route)
            else:
                route_length = 0

        route_length = int(route_length)

        if goal_state == "idle":
            return "idle"

        if goal_state == "sent":
            return "moving"

        if goal_state == "running":
            return "moving"

        if goal_state == "canceling":
            return "stopped"

        if goal_state == "done":
            if route_length > 0 and current_index >= route_length:
                return "finished"

            return "moving"

        # 혹시 이미 웹 상태로 들어오는 경우도 허용
        if goal_state in ["moving", "paused", "stopped", "finished"]:
            return goal_state

        return "idle"

    def _bt_status_callback(self, msg: String):
        """행동트리 상태 토픽을 구조별로 분류하여 Flask로 전달"""
        try:
            bt_data = json.loads(msg.data)
            headers = {"Content-Type": "application/json"}

            # 🛠️ [구조 분류 1] robot_status_state 처리 (1초 주기 데이터)
            if "robot_status" in bt_data:
                # 1초 주기로 들어오는 데이터는 상태 중복 체크 없이 '무조건' 전송해야 함
                
                url = f"{self.flask_base_url}/api/robot/status"  # Flask 측 로봇 상태 수신 주소
                
                res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
                
                if res.status_code != 200:
                    self.get_logger().error(f"robot_status_state 전송 실패 (HTTP: {res.status_code})")
                return

            # 🛠️ [구조 분류 2] navigation_state 처리 (이벤트성 변경 데이터)
            elif "status" in bt_data or "goal_state" in bt_data:
                web_status = self._convert_bt_goal_state_to_web_status(bt_data)
                
                current_index = int(bt_data.get("current_index",0))
                
                navigation_payload = {
                    "status": web_status,
                    "current_index": current_index
                }

                navigation_payload["bt_status_raw"] = bt_data.get("status")
                navigation_payload["bt_goal_state_raw"] = bt_data.get("goal_state")
                navigation_payload["is_paused"] = bt_data.get("is_paused", False)

                current_nav_key = (
                    navigation_payload["status"],
                    navigation_payload["current_index"],
                    navigation_payload["is_paused"]
                )

                if current_nav_key == self.last_nav_status:
                    return

                self.get_logger().info(
                    "\n" + "=" * 50 +
                    f"\n🔄 [내비게이션 상태 변환]" +
                    f"\nBT 원본: {json.dumps(bt_data, indent=2, ensure_ascii=False)}" +
                    f"\nWEB 변환: {json.dumps(navigation_payload, indent=2, ensure_ascii=False)}" +
                    "\n" + "=" * 50
                )

                url = f"{self.flask_base_url}/api/navigation/update"
                res = requests.post(
                    url,
                    data=json.dumps(navigation_payload),
                    headers=headers,
                    timeout=1.0
                )

                if res.status_code == 200:
                    self.get_logger().info(
                        f"Flask에 내비게이션 상태 변경 보고 성공: {navigation_payload}"
                    )
                    self.last_nav_status = current_nav_key
                else:
                    self.get_logger().error(
                        f"내비게이션 상태 보고 실패 (HTTP: {res.status_code})"
                    )
                            
        except Exception as e:
            self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")


def main(args=None):
    rclpy.init(args=args)
    from airport_guide.blackboard import Blackboard
    shared_blackboard = Blackboard()
    node = WebBridgeNode(blackboard=shared_blackboard)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()