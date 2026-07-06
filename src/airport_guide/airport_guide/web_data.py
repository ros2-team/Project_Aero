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
        
        ### 서버 주소
        self.flask_base_url = "http://192.168.0.9:5000" 
        ###
        self.last_handled_command_id = -1             
        self.polling_interval = 0.5                   
        
        # 🛠️ 수정한 규격에 맞게 내비게이션 상태만 변경 감지하기 위한 변수
        self.last_nav_status = None
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)

        # 행동트리 상태 모니터링
        self.bt_status_sub = self.create_subscription(
            String, "/robot/bt_status", self._bt_status_callback, 10
        )

        # (기존 코드 어딘가에 추가)
        self.full_path_sub = self.create_subscription(String, '/robot/full_path', self._full_path_callback, 10)

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
                        if has_command and not is_handled and (command_id > self.last_handled_command_id):
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
                                            "x": 0.6,
                                            "y": 0.0,
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
                                            "x": 0.0,
                                            "y": 0.0,
                                            "yaw": 0.0,
                                            "is_mid_point": True
                                        }
                                        processed_route.append(left_mid)
                                        self.get_logger().info(f"🔄 [Route Planner] {current_name} ↔ {next_name} (일반구간) -> Left 경유지 무조건 주입")

                            # self.blackboard.web_action = command_data.get("type")
                            # self.blackboard.web_route_list = processed_route
                            # self.blackboard.web_last_update_time = time.time()
                            # self.get_logger().info("📊 Planner가 보정한 데이터를 Blackboard 변수에 직대입 동기화 완료.")
                            
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
            "payload": {        # payload 안에 route 넣어서 패킹
                "route": command_data.get("route", [])
            }
        })
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
            elif "status" in bt_data:
                current_nav_status = bt_data.get("status")

                # 상태가 이전과 정확히 같으면 Flask 전송 패스
                if current_nav_status == self.last_nav_status:
                    return
                self.get_logger().info(
                    "\n" + "="*50 +
                    f"\n🔄 [내비게이션 상태 변경 감지] {self.last_nav_status} -> {current_nav_status}" +
                    f"\n{json.dumps(bt_data, indent=2, ensure_ascii=False)}" +
                    "\n" + "="*50
                )
                url = f"{self.flask_base_url}/api/navigation/update"
                res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
                if res.status_code == 200:
                    self.get_logger().info(f"Flask에 내비게이션 상태 변경 보고 성공: {bt_data}")
                    self.last_nav_status = current_nav_status
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