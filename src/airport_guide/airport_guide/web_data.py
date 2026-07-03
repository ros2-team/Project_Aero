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

                            raw_route = command_data.get("route", [])
                            processed_route = []

                            for wp in raw_route:
                                processed_route.append(wp)
                                
                                if wp.get("location_name") == "WayPoint_1":
                                    mid_wp = {
                                        "location_name": "Corner_Mid_Point",
                                        "x": 2.9,   
                                        "y": -0.4,  
                                        "is_mid_point": True  
                                    }
                                    processed_route.append(mid_wp)
                                    self.get_logger().info("🔄 [Route Planner] 'WayPoint_1' 감지 -> 직후에 코너링 우회용 중간 좌표를 계획 경로에 강제 주입했습니다.")

                            self.blackboard.web_action = command_data.get("type")
                            self.blackboard.web_route_list = processed_route
                            self.blackboard.web_last_update_time = time.time()
                            self.get_logger().info("📊 Planner가 보정한 데이터를 Blackboard 변수에 직대입 동기화 완료.")
                            
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