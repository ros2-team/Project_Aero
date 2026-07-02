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
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)
        
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
                        
                        if has_command and not is_handled and (command_id > self.last_handled_command_id):
                            self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_data.get('type')}")
                            
                            self.get_logger().info(
                                "\n" + "="*60 + 
                                f"\n📥 [WEB DATA RAW JSON] ID: {command_id}" +
                                f"\n🔹 전체 내용:\n{json.dumps(command_data, indent=2, ensure_ascii=False)}" +
                                "\n" + "="*60
                            )

                            # -------------------------------------------------------------------------
                            # 🎯 [Route Planner 레이어 기동] 중간 우회 좌표 동적 주입 인터셉터
                            # -------------------------------------------------------------------------
                            raw_route = command_data.get("route", [])
                            processed_route = []

                            for wp in raw_route:
                                # 1) 원본 경유지를 새 경로 버퍼에 순서대로 적재합니다.
                                processed_route.append(wp)
                                
                                # 2) 만약 들어온 경유지의 이름이 "WayPoint_1" 이라면 바로 뒤에 우회 좌표를 생성해 끼워 넣습니다.
                                if wp.get("location_name") == "WayPoint_1":
                                    mid_wp = {
                                        "location_name": "Corner_Mid_Point",
                                        "x": 2.9,   # 현장 장애물 맵 정보에 기반한 커스텀 우회 X 좌표
                                        "y": -0.4,  # 현장 장애물 맵 정보에 기반한 커스텀 우회 Y 좌표
                                        "is_mid_point": True  # 중간 경유지임을 구별하기 위해 추가한 커스텀 메타데이터 태그
                                    }
                                    processed_route.append(mid_wp)
                                    self.get_logger().info("🔄 [Route Planner] 'WayPoint_1' 감지 -> 직후에 코너링 우회용 중간 좌표를 계획 경로에 강제 주입했습니다.")

                            # 🎯 정제 완료된 전체 시퀀스 경로(processed_route)를 블랙보드에 최종 동기화합니다.
                            self.blackboard.web_action = command_data.get("type")
                            self.blackboard.web_route_list = processed_route
                            self.blackboard.web_last_update_time = time.time()
                            self.get_logger().info("📊 Planner가 보정한 데이터를 Blackboard 변수에 직대입 동기화 완료.")
                            
                            # 정제된 데이터를 통째로 변환해 행동트리에 토픽으로 사출
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
            "payload": {
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
        try:
            bt_data = json.loads(msg.data)
            url = f"{self.flask_base_url}/api/navigation/update"
            headers = {"Content-Type": "application/json"}
            
            res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
            if res.status_code == 200:
                self.get_logger().info(f"Flask에 로봇 실시간 상태 변경 보고 성공: {bt_data}")
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