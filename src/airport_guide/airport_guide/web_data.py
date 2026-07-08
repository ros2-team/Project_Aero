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
                                        self.get_logger().info(f"🔄 [Route Planner] {current_name} ↔ {next_name} (일반구간) -> Left 경유지 무조건 주입")
                            

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

        goal_state = str(bt_data.get("goal_state", bt_data.get("status", "idle"))).lower()
        current_index = int(bt_data.get("current_index", 0))
        
        # 기본적으로 로컬 캐시 길이를 보되, 비어있다면 BT가 보낸 데이터의 개수나 상수를 대안으로 사용
        route_length = len(self.active_processed_route)
        if route_length == 0:
            # bt_data에 route가 있다면 그것을 쓰고, 그것도 없다면 임시로 현재 인덱스를 기준으로 잡음
            route_length = len(bt_data.get("route", []))

        if goal_state == "idle": return "idle"
        if goal_state in ["sent", "running"]: return "moving"
        if goal_state == "canceling": return "stopped"
        if goal_state == "done":
            # 🚨 [핵심 방어] 경로 변수가 전부 비어있더라도, BT의 원본 인덱스가 4(최종) 이상이면 무조건 finished 처리
            if current_index >= 4 or (route_length > 0 and current_index >= route_length):
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

            # [구조 분류 2] navigation_state 처리 (이벤트성 변경 데이터)
            elif "status" in bt_data or "goal_state" in bt_data:
                current_index = int(bt_data.get("current_index", 0))
                goal_state = str(bt_data.get("goal_state", bt_data.get("status", ""))).lower()

                # 🛠️ [중간 경유지/일시정지 복합 제어 가드]
                is_mid_point_active = False

                # Case 1: 현재 가리키는 인덱스가 중간 경유지인 경우
                if current_index < len(self.active_processed_route):
                    if self.active_processed_route[current_index].get("is_mid_point", False):
                        is_mid_point_active = True

                # Case 2: 로봇이 도착해서 인덱스를 이미 다음 칸으로 올린 시점(done)인 경우 (직전 방 검사)
                if goal_state == "done" and current_index > 0:
                    prev_index = current_index - 1
                    if prev_index < len(self.active_processed_route):
                        if self.active_processed_route[prev_index].get("is_mid_point", False):
                            is_mid_point_active = True

                # 🎯 내부 상태를 웹 상태 규격으로 가공
                web_status = self._convert_bt_goal_state_to_web_status(bt_data)

                # 🚫 중간 경유지와 얽힌 상태라면 finished 출력을 차단하고 moving으로 강제 고정
                if is_mid_point_active:
                    self.get_logger().info("🚧 [중간경유지 가드 감지] 웹 상태를 moving으로 강제 유지합니다.")
                    web_status = "moving"

                ##################### kim-home ###################### 26/7/8 12:56 수정 
                web_display_index = 0
                for i in range(current_index):
                    # 현재 인덱스(current_index) 이전까지 지나온 경로 중, 중간 경유지가 아닌 '진짜 목적지' 개수만 카운트
                    if i < len(self.active_processed_route):
                        if not self.active_processed_route[i].get("is_mid_point", False):
                            web_display_index += 1
                            
                # 🚨 [추가된 핵심 가드] 주행이 완전히 끝났을(finished) 때 누락 방지!
                # 마지막 목적지가 카운트에서 빠지는 걸 막기 위해 '전체 진짜 목적지 개수'로 강제 보정.
                if web_status == "finished":
                    total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
                    web_display_index = total_real_dest

                navigation_payload = {
                    "status": web_status,
                    "current_index": web_display_index
                }

                ##################### yyj_test #################### 26/7/8 12:56 수정 
                # # 🛠️ 캐시 리스트가 정상적일 때만 카운트 연산 수행
                # if len(self.active_processed_route) > 0:
                #     web_display_index = 0
                #     for i in range(current_index):
                #         if i < len(self.active_processed_route):
                #             if not self.active_processed_route[i].get("is_mid_point", False):
                #                 web_display_index += 1
                                
                #     if web_status == "finished":
                #         total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
                #         web_display_index = total_real_dest
                # else:
                #     # 🚨 [동기화 꼬임 방어] 경로 리스트가 비어있다면 BT 원본 인덱스를 그대로 반영
                #     web_display_index = current_index

                # navigation_payload = {
                #     "status": web_status,
                #     "current_index": web_display_index
                # }

                navigation_payload["bt_status_raw"] = bt_data.get("status")
                navigation_payload["bt_goal_state_raw"] = bt_data.get("goal_state")
                navigation_payload["is_paused"] = bt_data.get("is_paused", False)

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

                    # 1️⃣ 사용자가 '일시정지'를 눌렀거나 풀었을 때 (최우선)
                    if last_paused != navigation_payload["is_paused"]:
                        should_send = True
                        trigger_reason = f"일시정지 상태 토글 ({last_paused} -> {navigation_payload['is_paused']})"

                    # 2️⃣ 진짜 목적지에 도착해서 인덱스가 갱신되었을 때!
                    elif last_index != navigation_payload["current_index"]:
                        should_send = True
                        trigger_reason = f"목적지 도착! 인덱스 갱신 ({last_index} -> {navigation_payload['current_index']})"

                    # 3️⃣ 최종 주행이 완전히 끝나서 상태가 'finished'가 되었을 때
                    elif last_status_str != "finished" and navigation_payload["status"] == "finished":
                        should_send = True
                        trigger_reason = "최종 목적지 도착 및 안내 종료"

                # 🎯 필터링 결과: 쏴야 할 때만 쏜다!
                if should_send:
                    # 🌟 [내가 추가 변형 반영] 전송 확정된 패킷만 깔끔하게 로그 출력
                    self.get_logger().info(
                        f"\n========================================================\n"
                        f"!!!!!!!!!!!!!!!!!!!!!!!!내가 추가!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                        f"📦 Payload:\n{json.dumps(navigation_payload, indent=2, ensure_ascii=False)}\n"
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
    # def _convert_bt_goal_state_to_web_status(self, bt_data: dict) -> str:
        
    #     # 행동트리 내부 상태를 웹 UI에서 사용하는 navigation status로 변환한다.

    #     # BT 내부 상태:
    #     # idle, sent, running, canceling, done

    #     # Web 상태:
    #     # idle, moving, paused, stopped, finished

    #     if bt_data.get("navigation_finished", False):
    #         return "finished"
        
    #     if bt_data.get("is_paused", False):
    #         return "paused"

    #     # 권장: 행동트리에서 goal_state로 보내는 경우
    #     goal_state = bt_data.get("goal_state")

    #     # 현재 코드 호환용: 행동트리에서 status로 보내는 경우도 처리
    #     if goal_state is None:
    #         goal_state = bt_data.get("status", "idle")

    #     goal_state = str(goal_state).lower()
    #     current_index = int(bt_data.get("current_index", 0))

    #     route_length = bt_data.get("route_length")
    #     if route_length is None:
    #         route = bt_data.get("route", [])
    #         if isinstance(route, list):
    #             route_length = len(route)
    #         else:
    #             route_length = 0

    #     route_length = int(route_length)

    #     if goal_state == "idle":
    #         return "idle"

    #     if goal_state == "sent":
    #         return "moving"

    #     if goal_state == "running":
    #         return "moving"

    #     if goal_state == "canceling":
    #         return "stopped"

    #     if goal_state == "done":
    #         if route_length > 0 and current_index >= route_length:
    #             return "finished"

    #         return "moving"

    #     # 혹시 이미 웹 상태로 들어오는 경우도 허용
    #     if goal_state in ["moving", "paused", "stopped", "finished"]:
    #         return goal_state

    #     return "idle"

    # # def _bt_status_callback(self, msg: String):
    #     """행동트리 상태 토픽을 구조별로 분류하여 Flask로 전달"""
    #     try:
    #         bt_data = json.loads(msg.data)
    #         headers = {"Content-Type": "application/json"}

    #         # [구조 분류 1] robot_status_state 처리 (1초 주기 데이터)
    #         if "robot_status" in bt_data:
    #             url = f"{self.flask_base_url}/api/robot/status"
    #             res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
    #             if res.status_code != 200:
    #                 self.get_logger().error(f"robot_status_state 전송 실패 (HTTP: {res.status_code})")
    #             return

    #         # [구조 분류 2] navigation_state 처리 (이벤트성 변경 데이터)
    #         elif "status" in bt_data or "goal_state" in bt_data:
    #             current_index = int(bt_data.get("current_index", 0))
    #             goal_state = str(bt_data.get("goal_state", bt_data.get("status", ""))).lower()

    #             # 🛠️ [중간 경유지/일시정지 복합 제어 가드]
    #             is_mid_point_active = False

    #             # Case 1: 현재 가리키는 인덱스가 중간 경유지인 경우
    #             if current_index < len(self.active_processed_route):
    #                 if self.active_processed_route[current_index].get("is_mid_point", False):
    #                     is_mid_point_active = True

    #             # Case 2: 로봇이 도착해서 인덱스를 이미 다음 칸으로 올린 시점(done)인 경우 (직전 방 검사)
    #             if goal_state == "done" and current_index > 0:
    #                 prev_index = current_index - 1
    #                 if prev_index < len(self.active_processed_route):
    #                     if self.active_processed_route[prev_index].get("is_mid_point", False):
    #                         is_mid_point_active = True

    #             # 🎯 내부 상태를 웹 상태 규격으로 가공
    #             web_status = self._convert_bt_goal_state_to_web_status(bt_data)

    #             # 🚫 중간 경유지와 얽힌 상태라면 finished 출력을 차단하고 moving으로 강제 고정
    #             if is_mid_point_active:
    #                 self.get_logger().info("🚧 [중간경유지 가드 감지] 웹 상태를 moving으로 강제 유지합니다.")
    #                 web_status = "moving"

    #             web_display_index = 0
    #             for i in range(current_index):
    #                 # 현재 인덱스(current_index) 이전까지 지나온 경로 중, 중간 경유지가 아닌 '진짜 목적지' 개수만 카운트
    #                 if i < len(self.active_processed_route):
    #                     if not self.active_processed_route[i].get("is_mid_point", False):
    #                         web_display_index += 1
                            
    #             # 🚨 [추가된 핵심 가드] 주행이 완전히 끝났을(finished) 때 누락 방지!
    #             if web_status == "finished":
    #                 total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
    #                 web_display_index = total_real_dest

    #             navigation_payload = {
    #                 "status": web_status,
    #                 "current_index": web_display_index
    #             }

    #             navigation_payload["bt_status_raw"] = bt_data.get("status")
    #             navigation_payload["bt_goal_state_raw"] = bt_data.get("goal_state")
    #             navigation_payload["is_paused"] = bt_data.get("is_paused", False)

    #             current_nav_key = (
    #                 navigation_payload["status"],
    #                 navigation_payload["current_index"],
    #                 navigation_payload["is_paused"]
    #             )

    #             # ================================================================
    #             # 🚀 [스마트 트리거] 진짜 필요한 3가지 순간에만 Flask로 쏜다!
    #             # ================================================================
    #             should_send = False
    #             trigger_reason = ""

    #             if self.last_nav_status is None:
    #                 should_send = True
    #                 trigger_reason = "최초 실행 동기화"
    #             else:
    #                 last_status_str = self.last_nav_status[0]
    #                 last_index = self.last_nav_status[1]
    #                 last_paused = self.last_nav_status[2]

    #                 # 1️⃣ 사용자가 '일시정지'를 눌렀거나 풀었을 때 (최우선)
    #                 if last_paused != navigation_payload["is_paused"]:
    #                     should_send = True
    #                     trigger_reason = f"일시정지 상태 토글 ({last_paused} -> {navigation_payload['is_paused']})"

    #                 # 2️⃣ 진짜 목적지에 도착해서 인덱스가 갱신되었을 때!
    #                 elif last_index != navigation_payload["current_index"]:
    #                     should_send = True
    #                     trigger_reason = f"목적지 도착! 인덱스 갱신 ({last_index} -> {navigation_payload['current_index']})"

    #                 # 3️⃣ 최종 주행이 완전히 끝나서 상태가 'finished'가 되었을 때
    #                 elif last_status_str != "finished" and navigation_payload["status"] == "finished":
    #                     should_send = True
    #                     trigger_reason = "최종 목적지 도착 및 안내 종료"

    #             # 🎯 필터링 결과: 쏴야 할 때만 쏜다!
    #             if should_send:
    #                 # 🌟 [내가 추가 변형 반영] 전송 확정된 패킷만 깔끔하게 로그 출력
    #                 self.get_logger().info(
    #                     f"\n========================================================\n"
    #                     f"!!!!!!!!!!!!!!!!!!!!!!!!내가 추가!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
    #                     f"📦 Payload:\n{json.dumps(navigation_payload, indent=2, ensure_ascii=False)}\n"
    #                     f"========================================================"
    #                 )

    #                 url = f"{self.flask_base_url}/api/navigation/update"
    #                 res = requests.post(url, data=json.dumps(navigation_payload), headers=headers, timeout=1.0)

    #                 if res.status_code == 200:
    #                     self.last_nav_status = current_nav_key
    #                 else:
    #                     self.get_logger().error(f"내비게이션 상태 보고 실패 (HTTP: {res.status_code})")
                        
    #     except Exception as e:
    #         self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")

    # def _bt_status_callback(self, msg: String):
    #     """행동트리 상태 토픽을 구조별로 분류하여 Flask로 전달"""
    #     try:
    #         bt_data = json.loads(msg.data)
    #         headers = {"Content-Type": "application/json"}

    #         # [구조 분류 1] robot_status_state 처리 (1초 주기 데이터)
    #         if "robot_status" in bt_data:
    #             url = f"{self.flask_base_url}/api/robot/status"
    #             res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
    #             if res.status_code != 200:
    #                 self.get_logger().error(f"robot_status_state 전송 실패 (HTTP: {res.status_code})")
    #             return

    #         # [구조 분류 2] navigation_state 처리 (이벤트성 변경 데이터)
    #         elif "status" in bt_data or "goal_state" in bt_data:
    #             current_index = int(bt_data.get("current_index", 0))
    #             goal_state = str(bt_data.get("goal_state", bt_data.get("status", ""))).lower()

    #             # 🛠️ [중간 경유지/일시정지 복합 제어 가드]
    #             is_mid_point_active = False

    #             # Case 1: 현재 가리키는 인덱스가 중간 경유지인 경우
    #             if current_index < len(self.active_processed_route):
    #                 if self.active_processed_route[current_index].get("is_mid_point", False):
    #                     is_mid_point_active = True

    #             # Case 2: 로봇이 도착해서 인덱스를 이미 다음 칸으로 올린 시점(done)인 경우 (직전 방 검사)
    #             if goal_state == "done" and current_index > 0:
    #                 prev_index = current_index - 1
    #                 if prev_index < len(self.active_processed_route):
    #                     if self.active_processed_route[prev_index].get("is_mid_point", False):
    #                         is_mid_point_active = True

    #             # 🎯 내부 상태를 웹 상태 규격으로 가공
    #             web_status = self._convert_bt_goal_state_to_web_status(bt_data)

    #             # 🚫 중간 경유지와 얽힌 상태라면 finished 출력을 차단하고 moving으로 강제 고정
    #             if is_mid_point_active:
    #                 self.get_logger().info("🚧 [중간경유지 가드 감지] 웹 상태를 moving으로 강제 유지합니다.")
    #                 web_status = "moving"

                # web_display_index = 0
                # for i in range(current_index):
                #     # 현재 인덱스(current_index) 이전까지 지나온 경로 중, 중간 경유지가 아닌 '진짜 목적지' 개수만 카운트
                #     if i < len(self.active_processed_route):
                #         if not self.active_processed_route[i].get("is_mid_point", False):
                #             web_display_index += 1
                            
                # # 🚨 [추가된 핵심 가드] 주행이 완전히 끝났을(finished) 때 누락 방지!
                # # 마지막 목적지가 카운트에서 빠지는 걸 막기 위해 '전체 진짜 목적지 개수'로 강제 보정.
                # if web_status == "finished":
                #     total_real_dest = sum(1 for wp in self.active_processed_route if not wp.get("is_mid_point", False))
                #     web_display_index = total_real_dest

                # navigation_payload = {
                #     "status": web_status,
                #     "current_index": web_display_index
                # }

    #             navigation_payload["bt_status_raw"] = bt_data.get("status")
    #             navigation_payload["bt_goal_state_raw"] = bt_data.get("goal_state")
    #             navigation_payload["is_paused"] = bt_data.get("is_paused", False)

    #             current_nav_key = (
    #                 navigation_payload["status"],
    #                 navigation_payload["current_index"],
    #                 navigation_payload["is_paused"]
    #             )
                # # ================================================================
                # # 🚀 [스마트 트리거] 진짜 필요한 3가지 순간에만 Flask로 쏜다!
                # # 4번 다다닥 바뀌는 잉여 상태는 여기서 전부 씹어버립니다.
                # # 0708 10:46 새로운 코드 new -> 도착시 이전 인덱스와 비교하여 1번만 전송
                # # ================================================================
                # should_send = False
                # trigger_reason = ""

                # if self.last_nav_status is None:
                #     should_send = True
                #     trigger_reason = "최초 실행 동기화"
                # else:
                #     last_status_str = self.last_nav_status[0]
                #     last_index = self.last_nav_status[1]
                #     last_paused = self.last_nav_status[2]

                #     # 1️⃣ 사용자가 '일시정지'를 눌렀거나 풀었을 때 (최우선)
                #     if last_paused != navigation_payload["is_paused"]:
                #         should_send = True
                #         trigger_reason = f"일시정지 상태 토글 ({last_paused} -> {navigation_payload['is_paused']})"

                #     # 2️⃣ 진짜 목적지에 도착해서 인덱스가 갱신되었을 때!
                #     # (가짜 중간 경유지는 계산에서 빠져있으므로 알아서 무시됨)
                #     elif last_index != navigation_payload["current_index"]:
                #         should_send = True
                #         trigger_reason = f"목적지 도착! 인덱스 갱신 ({last_index} -> {navigation_payload['current_index']})"

                #     # 3️⃣ 최종 주행이 완전히 끝나서 상태가 'finished'가 되었을 때
                #     elif last_status_str != "finished" and navigation_payload["status"] == "finished":
                #         should_send = True
                #         trigger_reason = "최종 목적지 도착 및 안내 종료"


                # # 🎯 필터링 결과: 쏴야 할 때만 쏜다!
                # if should_send:
                #     self.get_logger().info(
                #         "\n" + "=" * 50 +
                #         f"\n🎯 [웹 전송 트리거 발동] 사유: {trigger_reason}" +
                #         f"\nWEB 변환: {json.dumps(navigation_payload, indent=2, ensure_ascii=False)}" +
                #         "\n" + "=" * 50
                #     )

                #     url = f"{self.flask_base_url}/api/navigation/update"
                #     res = requests.post(url, data=json.dumps(navigation_payload), headers=headers, timeout=1.0)

                #     if res.status_code == 200:
                #         # 갓벽하게 성공했을 때만 현재 상태를 업데이트 (다음 비교를 위해)
                #         self.last_nav_status = current_nav_key
                #     else:
                #         self.get_logger().error(f"내비게이션 상태 보고 실패 (HTTP: {res.status_code})")
                        
                # else:
                #     # 목적지 도착이 아닌 BT 내부의 자잘한 상태 변화(done->idle->sent)는 조용히 무시 (씹음)
                #     pass

                ############ 0708 10:45 기존 코드 ###############

                # if current_nav_key == self.last_nav_status:
                #     return

                # self.get_logger().info(
                #     "\n" + "=" * 50 +
                #     f"\n🔄 [내비게이션 상태 변환]" +
                #     f"\nBT 원본: {json.dumps(bt_data, indent=2, ensure_ascii=False)}" +
                #     f"\nWEB 변환: {json.dumps(navigation_payload, indent=2, ensure_ascii=False)}" +
                #     "\n" + "=" * 50
                # )

                # url = f"{self.flask_base_url}/api/navigation/update"
                # res = requests.post(
                #     url,
                #     data=json.dumps(navigation_payload),
                #     headers=headers,
                #     timeout=1.0
                # )

                # if res.status_code == 200:
                #     self.get_logger().info(
                #         f"Flask에 내비게이션 상태 변경 보고 성공: {navigation_payload}"
                #     )
                #     self.last_nav_status = current_nav_key
                # else:
                #     self.get_logger().error(
                #         f"내비게이션 상태 보고 실패 (HTTP: {res.status_code})"
                #     )
                            
        except Exception as e:
            self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")

    ######################### 받은 전체경로 웹으로 보내는 함수 ####################### 26/7/8 10:55 추가
    def _full_path_callback(self, msg):
        """전처리 노드가 만든 전체 경로 JSON을 받아서 Flask로 쏴주는 함수"""
        try:
            # 1. 받은 토픽 데이터를 딕셔너리로 변환
            path_data = json.loads(msg.data)
            # self.get_logger().info(f"path_data 값은 {path_data} 입니다. !!!!!!!!!!!!!!!!!!!!")
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