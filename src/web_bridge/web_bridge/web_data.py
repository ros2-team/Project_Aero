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

        self.last_raw_current_index = 0

        # 7/7 현재 주행 중인 보정 경로 데이터를 보존하기 위한 로컬 캐시 추가
        self.active_processed_route = []

        # 7/8 전체 path값 sub & callback 함수 추가
        self.full_path_sub = self.create_subscription(String, '/robot/full_path', self._full_path_callback, 10)

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
                                
                            ######################## 경로 보정 최신화 26/7/8 18:02 ###########################
                            if has_command and not is_handled and (command_id != self.last_handled_command_id):
                                command_type = command_data.get('type')
                                self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_type}")

                                # 🛠️ [핵심 수정 1] 주행 제어 명령일 경우, 경로 보정 로직을 타지 않고 기존 캐시를 유지합니다.
                                if command_type in ["pause_navigation", "resume_navigation", "stop_navigation"]:
                                    self.get_logger().info(f"⏸️ 제어 명령({command_type}) 수신. 기존 경로 캐시를 유지합니다.")
                                    
                                    # 경로 보정을 생략하고, 행동트리에 명령만 바로 쏴줍니다.
                                    self._publish_to_behavior_tree(command_data)
                                    self._mark_command_as_handled(command_id)
                                    self.last_handled_command_id = command_id
                                    continue  # 아래의 경로 덮어쓰기 로직을 건너뛰고 다음 폴링으로!
                                # ---------------------------------------------------------------------
                                # qr 
                                # ---------------------------------------------------------------------
                                elif command_type == "qr_call_navigation":
                                    # Flask가 보낸 route 배열에서 첫 번째 목적지 추출
                                    flask_route = command_data.get("route", [])
                                    
                                    if flask_route:
                                        qr_wp = flask_route[0]
                                        self.get_logger().info(f"[QR 호출 감지] 목적지: {qr_wp.get('location_name')}")
                                        
                                        # 로봇 행동트리가 최종 인식할 수 있는 내부 규격으로 재가공
                                        qr_route = [{
                                            "location_code": qr_wp.get("location_code"),
                                            "location_name": qr_wp.get("location_name"),
                                            "order": 1,
                                            "x": float(qr_wp.get("x", 0.0)),
                                            "y": float(qr_wp.get("y", 0.0)),
                                            "yaw": float(qr_wp.get("yaw", 0.0)),
                                            "is_mid_point": False
                                        }]
                                        
                                        # 가공된 루트를 command_data에 덮어쓰기하여 하부 로직으로 안전하게 전달
                                        command_data["route"] = qr_route
                                        command_data["location_name"] = qr_wp.get("location_name")
                                    else:
                                        self.get_logger().error("❌ [QR 에러] QR 호출 명령에 route 데이터가 유실되었습니다.")
                                        continue


                            ############## 경로 보정 ###############
                            raw_route = command_data.get("route", [])
                            processed_route = []

                            # 🌟 1. 현재 로봇의 출발 위치 이름을 가져옵니다. (위쪽 코드 스타일과 통일)
                            current_location = command_data.get("location_name", "대기소") 

                            # 🚀 2. 맨 처음 출발할 때 선행 경유지 주입 로직
                            if raw_route:
                                first_wp = raw_route[0]
                                first_name = first_wp.get("location_name")
                                first_order = first_wp.get("order", 0)

                                # 🚨 [예외 처리] 현재 위치가 A고 첫 목적지가 B (또는 B->A)인 직선구간 프리패스!
                                if (current_location == "게이트 A" and first_name == "게이트 B") or \
                                (current_location == "게이트 B" and first_name == "게이트 A"):
                                    
                                    self.get_logger().info(f"🚀 [Route Planner] 출발지({current_location}) ↔ 첫 목적지({first_name}) : 직선구간 프리패스! (선행 경유지 생략)")
                                    # 아무 경유지도 넣지 않고 바로 첫 목적지로 직행합니다.

                                # 일반적인 출발 상황 (첫 목적지가 A/B면 Right, 나머지는 Left)
                                elif first_name in ["게이트 A", "게이트 B", "게이트 C"]:
                                    initial_mid = {
                                        "location_code": "MID_RIGHT",
                                        "location_name": "Corner_Right_Mid",
                                        "order": first_order, 
                                        "x": 0.44,
                                        "y": 0.08,
                                        "yaw": 0.0,
                                        "is_mid_point": True
                                    }
                                    processed_route.append(initial_mid)
                                    self.get_logger().info(f"🚩 [Route Planner] 출발지({current_location}) -> {first_name} : Right 경유지 먼저 거치고 출발!")
                                    
                                else:
                                    initial_mid = {
                                        "location_code": "MID_LEFT",
                                        "location_name": "Corner_Left_Mid",
                                        "order": first_order,
                                        "x": -0.37,
                                        "y": -0.08,
                                        "yaw": 0.0,
                                        "is_mid_point": True
                                    }
                                    processed_route.append(initial_mid)
                                    self.get_logger().info(f"🚩 [Route Planner] 출발지({current_location}) -> {first_name} : Left 경유지 먼저 거치고 출발!")


                            # 🔄 3. 기존 로직: 경로 리스트를 돌면서 중간 경유지 자동 주입
                            for i in range(len(raw_route)):
                                current_wp = raw_route[i]
                                processed_route.append(current_wp)  # 목적지 추가
                                
                                # 마지막 목적지가 아니라면, '현재 목적지'와 '다음 목적지' 사이의 이동을 검사
                                if i < len(raw_route) - 1:
                                    current_name = current_wp.get("location_name")
                                    next_name = raw_route[i+1].get("location_name")
                                    current_order = current_wp.get("order", 0)
                                    
                                    # 🚦 Gate A ↔ C 또는 Gate B ↔ C 구간인 경우 -> Right 경유지 주입
                                    if (current_name == "게이트 A" and next_name == "게이트 C") or \
                                    (current_name == "게이트 C" and next_name == "게이트 A") or \
                                    (current_name == "게이트 B" and next_name == "게이트 C") or \
                                    (current_name == "게이트 C" and next_name == "게이트 B"):
                                        
                                        right_mid = {
                                            "location_code": "MID_RIGHT",   
                                            "location_name": "Corner_Right_Mid",
                                            "order": current_order,          
                                            "x": 0.44,
                                            "y": 0.08,
                                            "yaw": 0.0,                     
                                            "is_mid_point": True
                                        }
                                        processed_route.append(right_mid)
                                        self.get_logger().info(f"🔄 [Route Planner] {current_name} ↔ {next_name} (특수구간) -> Right 경유지 강제 주입")

                                    # 🚦 게이트 A ↔ 게이트 B 구간인 경우 -> 경유지 없이 프리패스!
                                    elif (current_name == "게이트 A" and next_name == "게이트 B") or \
                                        (current_name == "게이트 B" and next_name == "게이트 A"):
                                        
                                        self.get_logger().info(f"🚀 [Route Planner] {current_name} ↔ {next_name} (직선구간) -> 경유지 패스, 최단거리 직행!")
                                        pass 
                                        
                                    # 🚦 화장실, 면세점 등 그 외의 모든 구간 -> 무조건 Left 경유지 주입
                                    else:
                                        left_mid = {
                                            "location_code": "MID_LEFT",
                                            "location_name": "Corner_Left_Mid",
                                            "order": current_order,
                                            "x": -0.37,
                                            "y": -0.08,
                                            "yaw": 0.0,
                                            "is_mid_point": True
                                        }
                                        processed_route.append(left_mid)

                            # 7/7 가공 완료된 경로를 필터링 판정용 멤버 변수에 캐싱
                            self.active_processed_route = processed_route
                            
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

    ########################### 7/8 17:25 new 버전 ########################
    def _convert_bt_goal_state_to_web_status(self, bt_data: dict, current_index: int) -> str:
            if bt_data.get("navigation_finished", False):
                return "finished"
            if bt_data.get("is_paused", False):
                return "paused"

            goal_state = str(bt_data.get("goal_state", bt_data.get("status", "idle"))).lower()
            route_length = len(self.active_processed_route)

            if goal_state == "idle": return "idle"
            if goal_state in ["sent", "running"]: return "moving"
            if goal_state == "canceling": return "stopped"
            if goal_state == "done":

                # 원본 백업 인덱스로 최종 목적지 도달 여부 판단
                if route_length > 0 and current_index >= route_length:
                    return "finished"
                return "moving"
            
            if goal_state in ["moving", "paused", "stopped", "finished"]:
                return goal_state
            
            return "idle"

    def _bt_status_callback(self, msg: String):
        """행동트리 상태 토픽을 구조별로 분류하여 Flask로 전달"""
        try:
            bt_data = json.loads(msg.data)
            headers = {"Content-Type": "application/json"}

            # [구조 분류 1] robot_status_state 처리 (1초 주기 데이터)
            if "robot_status" in bt_data:
                url = f"{self.flask_base_url}/api/robot/status"
                res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)

                if res.status_code != 200:
                    self.get_logger().error(f"robot_status_state 전송 실패 (HTTP: {res.status_code})")

                return

            elif "status" in bt_data or "goal_state" in bt_data:
                
                # -------------------------------------------------------------
                # 🛠️ [수정됨] 일시정지 가드 조건 완화 및 정확한 복구
                # -------------------------------------------------------------
                current_index = int(bt_data.get("current_index", 0))
                is_paused = bt_data.get("is_paused", False)

                # 무조건 is_paused라고 덮어씌우는 게 아니라, 진짜로 인덱스가 0으로 누락됐을 때만 복구합니다.
                if is_paused and current_index == 0 and self.last_raw_current_index > 0:
                    self.get_logger().warn(f"⚠️ [일시정지 가드] BT 인덱스 누락 감지 -> {self.last_raw_current_index}(으)로 복구")
                    current_index = self.last_raw_current_index

                elif not is_paused and current_index > 0:
                    # 정상 주행 중이고 유효한 인덱스일 때만 백업
                    self.last_raw_current_index = current_index

                goal_state = str(bt_data.get("goal_state", bt_data.get("status", ""))).lower()

                # 🛠️ [중간 경유지/일시정지 복합 제어 가드]
                is_mid_point_active = False

                # Case 1: 현재 가리키는 인덱스가 중간 경유지인 경우
                if current_index < len(self.active_processed_route):
                    if self.active_processed_route[current_index].get("is_mid_point", False):
                        is_mid_point_active = True

                # Case 2: 로봇이 도착해서 인덱스를 이미 다음 칸으로 올린 시점(done)인 경우
                if goal_state == "done" and current_index > 0:
                    prev_index = current_index - 1
                    if prev_index < len(self.active_processed_route):
                        if self.active_processed_route[prev_index].get("is_mid_point", False):
                            is_mid_point_active = True

                # 🎯 내부 상태를 웹 상태 규격으로 가공 (안전하게 복구된 인덱스 전달)
                web_status = self._convert_bt_goal_state_to_web_status(bt_data, current_index)
                
                # 🛠️ [핵심 수정 2] 일시정지("paused") 상태가 아닐 때만 중간 경유지 가드를 작동시킵니다.
                if is_mid_point_active and web_status != "paused":
                    web_status = "moving"
  
                # -------------------------------------------------------------
                # 🛠️ [복구 로직 추가] 노드 재시작 등으로 경로 캐시가 날아갔을 때 BT 원본으로 복구
                # (빈 배열 때문에 인덱스가 오작동하여 도착으로 처리되는 현상 방지)
                # -------------------------------------------------------------
                if len(self.active_processed_route) == 0:
                    bt_route = bt_data.get("route", [])
                    if len(bt_route) > 0:
                        self.active_processed_route = bt_route
                        self.get_logger().info("🔄 [Web Bridge] 경로 캐시 소실 감지 -> BT 원본 데이터로 복구 완료")

                # 진짜 목적지 디스플레이 인덱스 계산
                web_display_index = 0
                for i in range(current_index):
                    if i < len(self.active_processed_route):
                        if not self.active_processed_route[i].get("is_mid_point", False):
                            web_display_index += 1
                            
                if web_status == "finished":
                    if len(self.active_processed_route) > 0:
                        total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
                        web_display_index = total_real_dest

                navigation_payload = {
                    "status": web_status,
                    "current_index": web_display_index,
                    "bt_status_raw": bt_data.get("status"),
                    "bt_goal_state_raw": bt_data.get("goal_state"),
                    "is_paused": is_paused
                }

                current_nav_key = (
                    navigation_payload["status"],
                    navigation_payload["current_index"],
                    navigation_payload["is_paused"]
                )

                # ================================================================
                # 🚀 [스마트 트리거] 진짜 필요한 3가지 순간에만 Flask로 쏜다!
                # ================================================================
                should_send = False
                trigger_reason = ""

                if self.last_nav_status is None:
                    should_send = True
                    trigger_reason = "최초 실행 동기화"
                else:
                    last_status_str = self.last_nav_status[0]
                    last_index = self.last_nav_status[1]
                    last_paused = self.last_nav_status[2]

                    if last_paused != navigation_payload["is_paused"]:
                        should_send = True
                        trigger_reason = f"일시정지 상태 토글 ({last_paused} -> {navigation_payload['is_paused']})"
                    elif last_index != navigation_payload["current_index"]:
                        should_send = True
                        trigger_reason = f"목적지 도착! 인덱스 갱신 ({last_index} -> {navigation_payload['current_index']})"
                    elif last_status_str != "finished" and navigation_payload["status"] == "finished":
                        if bt_data.get("navigation_finished", False) or current_index >= len(self.active_processed_route):
                            should_send = True
                            trigger_reason = "최종 목적지 도착 및 안내 종료"

                # 🚨 [방화벽 로직 수정 - 타이밍 밀림 완벽 해결]
                # 중간 경유지(코너)를 도는 중이라도, 방금 진짜 목적지에 도착해서 인덱스가 오른 거라면
                # 절대 차단하지 않고 웹으로 즉시 쏴서 UI 도착 불빛이 즉각 켜지도록 예외 처리합니다.
                if is_mid_point_active and not is_paused:
                    if self.last_nav_status is not None and last_index == navigation_payload["current_index"]:
                        # 인덱스 변화도 없는데 자잘하게 바뀌는 상태(노이즈)만 차단
                        should_send = False
                    else:
                        # 인덱스가 변했다면(방금 도착) 차단하지 않고 그대로 통과 (should_send = True 유지)
                        pass

                # 🎯 필터링 결과: 쏴야 할 때만 쏜다!
                if should_send:
                    self.get_logger().info(
                        f"\n========================================================\n"
                        f"📦 Payload (이유: {trigger_reason}):\n{json.dumps(navigation_payload, indent=2, ensure_ascii=False)}\n"
                        f"========================================================"
                    )

                    url = f"{self.flask_base_url}/api/navigation/update"
                    res = requests.post(url, data=json.dumps(navigation_payload), headers=headers, timeout=1.0)

                    if res.status_code == 200:
                        self.last_nav_status = current_nav_key
                    else:
                        self.get_logger().error(f"내비게이션 상태 보고 실패 (HTTP: {res.status_code})")
                    
        except Exception as e:
            self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")
   
    def _full_path_callback(self, msg):
        """전처리 노드가 만든 전체 경로 JSON을 받아서 Flask로 쏴주는 함수"""
        try:
            # 1. 받은 토픽 데이터를 딕셔너리로 변환
            path_data = json.loads(msg.data)
            # 2. Flask API 주소 (환경에 맞게 IP와 포트 수정 필요!)
            # 만약 DB가 있는 교육원 PC 주소가 192.168.1.100 이라면 거기로 맞춰야 합니다.
            flask_api_url = f"{self.flask_base_url}/api/navigation/path"
            
            # 3. HTTP POST 요청으로 프론트엔드에 쏴주기
            headers = {'Content-Type': 'application/json'}
            response = requests.post(flask_api_url, json=path_data, headers=headers, timeout=2.0)
            
            if response.status_code == 200:
                self.get_logger().info("✅ [Web Bridge] 프론트엔드로 전체 경로(Path) 전송 성공!")
            else:
                self.get_logger().error(f"❌ [Web Bridge] 프론트 전송 실패 (상태 코드: {response.status_code})")
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"🌐 [Web Bridge] Flask 서버와 통신할 수 없습니다: {e}")
            
        except Exception as e:
            self.get_logger().error(f"⚠️ [Web Bridge] 경로 데이터 처리 중 에러: {e}")

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