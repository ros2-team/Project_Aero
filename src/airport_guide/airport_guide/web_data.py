#!/usr/bin/env python3
import time
import requests
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WebBridgeNode(Node):
    # 🎯 [수정] 외부 메인 실행부(main)로부터 공유 blackboard 객체를 주입받도록 변경
    def __init__(self, blackboard):
        super().__init__("web_bridge_node")
        self.blackboard = blackboard # 공유 데이터 저장소 연결
        
        # -----------------------------------------------------------------------------------------
        # [설정 및 전역 변수 관리]
        # -----------------------------------------------------------------------------------------
        self.flask_base_url = "http://192.168.0.9:5000" # Flask 서버 주소 (필요 시 IP 변경)
        self.last_handled_command_id = -1             # 중복 처리 방지를 위한 마지막 처리 명령 ID 저장
        self.polling_interval = 0.5                   # Flask 서버를 조회할 주기 (0.5초)

        # 하위 로봇 제어단(행동트리 preprocessor)으로 정제된 명령을 던질 ROS2 퍼블리셔 (기존 유지)
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)
        
        # 하위 로봇단(행동트리)으로부터 실시간 상태를 피드백받아 Flask로 쏠 ROS2 서브스크라이버
        self.bt_status_sub = self.create_subscription(
            String, "/robot/bt_status", self._bt_status_callback, 10
        )

        self.get_logger().info("확정 API 기반 ROS2 Web Bridge Node 가동 시작.")

        # -----------------------------------------------------------------------------------------
        # [핵심] Flask 서버에서 명령을 주기적으로 꺼내오기(Pull) 위한 타이머 스레드 가동
        # -----------------------------------------------------------------------------------------
        self.polling_thread = threading.Thread(target=self._command_polling_loop, daemon=True)
        self.polling_thread.start()

    # =========================================================================================
    # 1. 꺼내오기 흐름: GET /api/robot/command  ->  POST /api/robot/command/handled
    # =========================================================================================
    def _command_polling_loop(self):
        """설정된 주기마다 Flask 서버를 폴링하며 새로운 명령이 생성되었는지 감시하는 루프"""
        while rclpy.ok():
            try:
                # [1단계] Flask 서버에 최신 로봇 명령 조회 요청 (GET)
                response = requests.get(f"{self.flask_base_url}/api/robot/command", timeout=1.0)
                
                if response.status_code == 200:
                    res_data = response.json()
                    
                    if res_data.get("status") == "success":
                        command_data = res_data.get("command", {})
                        
                        # [2단계] 조건 검증: 새 명령이 존재하고(has_command), 아직 처리되지 않았으며(is_handled == False),
                        #         최근에 처리했던 command_id보다 높은 새로운 명령인지 확인
                        has_command = command_data.get("has_command", False)
                        is_handled = command_data.get("is_handled", True)
                        command_id = command_data.get("command_id", -1)

                        if has_command and not is_handled and (command_id > self.last_handled_command_id):
                            self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_data.get('type')}")
                            
                            # 🎯 [로그 추가] 터미널에 웹에서 온 원본 JSON 데이터를 통째로 예쁘게 출력
                            self.get_logger().info(
                                "\n" + "="*60 + 
                                f"\n📥 [WEB DATA RAW JSON] ID: {command_id}" +
                                f"\n🔹 전체 내용:\n{json.dumps(command_data, indent=2, ensure_ascii=False)}" +
                                "\n" + "="*60
                            )

                            # 🎯 순수하게 블랙보드의 웹 데이터 영역 변수값만 업데이트 (동기화)
                            self.blackboard.web_action = command_data.get("type")
                            self.blackboard.web_route_list = command_data.get("route", [])
                            self.blackboard.web_last_update_time = time.time()
                            self.get_logger().info("📊 웹 원본 데이터를 Blackboard 변수에 직대입 동기화 완료.")

                            # [3단계] 검증 통과 시 해당 데이터를 그대로 행동트리가 읽는 ROS2 토픽으로 전달(Publish)
                            self._publish_to_behavior_tree(command_data)
                            
                            # [4단계] 전달 성공 후, Flask 서버에 이 command_id를 처리 완료했다고 보고 (POST)
                            self._mark_command_as_handled(command_id)
                            
                            # 최근 처리 ID 갱신하여 중복 실행 방지
                            self.last_handled_command_id = command_id

            except requests.exceptions.RequestException as e:
                # 웹 서버가 꺼져있거나 네트워크 요동으로 인한 예외 발생 시 에러 로그만 찍고 버팀
                self.get_logger().error(f"Flask 서버 폴링 중 통신 실패: {e}")
            
            # 설정된 주기만큼 대기 후 다음 조회 진행
            time.sleep(self.polling_interval)

            
            if has_command and not is_handled and (command_id > self.last_handled_command_id):
                            self.get_logger().info(f"[새 명령 감지] ID: {command_id}, Type: {command_data.get('type')}")
                            
                            # 🎯 [여기에 삽입] 터미널에 웹에서 온 원본 JSON 데이터를 통째로 예쁘게 출력
                            self.get_logger().info(
                                "\n" + "="*60 + 
                                f"\n📥 [WEB DATA RAW JSON] ID: {command_id}" +
                                f"\n🔹 전체 내용:\n{json.dumps(command_data, indent=2, ensure_ascii=False)}" +
                                "\n" + "="*60
                            )

                            # [추가] 순수하게 블랙보드의 웹 데이터 영역 변수값만 업데이트 (동기화)
                            self.blackboard.web_action = command_data.get("type")
                            self.blackboard.web_route_list = command_data.get("route", [])
                            self.blackboard.web_last_update_time = time.time()



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
        """[POST /api/robot/command/handled] 명령 처리 완료 알림 API 호출"""
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

    # =========================================================================================
    # 2. 업로드 흐름: 행동트리 피드백 -> POST /api/navigation/update
    # =========================================================================================
    def _bt_status_callback(self, msg: String):
        """로봇 하위(행동트리)에서 상태 변경 토픽이 올라오면 Flask로 즉시 보고하는 콜백 함수"""
        try:
            bt_data = json.loads(msg.data)
            url = f"{self.flask_base_url}/api/navigation/update"
            headers = {"Content-Type": "application/json"}
            
            res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
            if res.status_code == 200:
                self.get_logger().info(f"Flask에 로봇 실시간 상태 변경 보고 성공: {bt_data}")
        except Exception as e:
            self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")


# =========================================================================================
# 메인 실행부
# =========================================================================================
def main(args=None):
    rclpy.init(args=args)
    
    # 🎯 [테스트 및 독립 구동용] main을 단독 실행할 때도 Blackboard를 주입받도록 인스턴스 생성 처리
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