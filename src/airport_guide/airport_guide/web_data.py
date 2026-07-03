#!/usr/bin/env python3
import time
import requests
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathThroughPoses
from geometry_msgs.msg import PoseStamped

class WebBridgeNode(Node):
    def __init__(self, blackboard):
        super().__init__("web_bridge_node")
        self.blackboard = blackboard 
        ### path값 다 받아오기
        self.path_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
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
                            # 🎯 [Route Planner 레이어] 지정된 5개 목적지 직후 우회 좌표 동적 주입
                            # -------------------------------------------------------------------------
                            raw_route = command_data.get("route", [])
                            processed_route = []

                            # 다중 코너 보정 타깃 리스트 정의
                            target_locations = ["게이트 A", "게이트 B", "게이트 C", "화장실", "면세점"]

                            for wp in raw_route:
                                # 1) 원본 경유지를 정제 리스트에 순서대로 먼저 적재
                                processed_route.append(wp)
                                
                                # 2) 수신된 목적지가 타깃 목록에 포함되어 있는지 검사
                                if wp.get("location_name") in target_locations:
                                    mid_wp = {
                                        "location_name": "Corner_Mid_Point",
                                        "x": 0.1,   # 현장 환경에 맞춰 수정한 코너링 회피 X 좌표
                                        "y": 0.1,  # 현장 환경에 맞춰 수정한 코너링 회피 Y 좌표
                                        "is_mid_point": True  # ArrivalNode에서 무정차 식별용 플래그
                                    }
                                    processed_route.append(mid_wp)
                                    self.get_logger().info(f"🔄 [Route Planner] '{wp.get('location_name')}' 감지 -> 코너링 우회용 중간 좌표를 경로 시퀀스에 강제 주입했습니다.")
                                    
                                    ### [STEP 1] 완성된 경로를 Nav2에 보내서 선(Path)으로 그려달라고 요청하기
                                    
                            poses_for_nav2 = []
                            for p_wp in processed_route:
                                pose = PoseStamped()
                                pose.header.frame_id = 'map'
                                pose.header.stamp = self.get_clock().now().to_msg()
                                pose.pose.position.x = float(p_wp.get("x", 0.0))
                                pose.pose.position.y = float(p_wp.get("y", 0.0))
                                poses_for_nav2.append(pose)

                            if not self.path_client.wait_for_server(timeout_sec=2.0):
                                self.get_logger().error("❌ Nav2 ComputePathThroughPoses 서버 응답 없음! (로봇 켜져있음?)")
                            else:
                                goal_msg = ComputePathThroughPoses.Goal()
                                goal_msg.goals = poses_for_nav2
                                
                                # 비동기로 목표 던지고, 응답 오면 콜백 함수 실행하도록 연결
                                send_goal_future = self.path_client.send_goal_async(goal_msg)
                                send_goal_future.add_done_callback(self._path_goal_response_callback)
                                self.get_logger().info("🚀 Nav2에 전체 Global Path 계산 요청 전송 완료!")


                            # 🎯 전처리가 완결된 전체 시퀀스 경로를 블랙보드에 최종 동기화
                            self.blackboard.web_action = command_data.get("type")
                            self.blackboard.web_route_list = processed_route
                            self.blackboard.web_last_update_time = time.time()
                            self.get_logger().info("📊 Planner가 보정한 데이터를 Blackboard 변수에 직대입 동기화 완료.")
                            
                            # 정제된 데이터를 반영하여 하위 행동트리 토픽으로 사출
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

    # def _bt_status_callback(self, msg: String):
    #     try:
    #         bt_data = json.loads(msg.data)
    #         url = f"{self.flask_base_url}/api/navigation/update"
    #         headers = {"Content-Type": "application/json"}
            
    #         res = requests.post(url, data=json.dumps(bt_data), headers=headers, timeout=1.0)
    #         if res.status_code == 200:
    #             self.get_logger().info(f"Flask에 로봇 실시간 상태 변경 보고 성공: {bt_data}")
    #     except Exception as e:
    #         self.get_logger().error(f"로봇 상태 업데이트 보고 중 오류 발생: {e}")

    def _send_path_to_flask(self, path):
        """Nav2에서 연산된 전체 Global Path를 수신하여 Flask 서버로 HTTP POST 전송"""
        try:
            # 1. ROS Path 메시지에서 x, y 좌표만 빼서 리스트로 묶기
            path_coords = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in path.poses]
            
            # 2. 웹 서버로 보낼 JSON 페이로드(Payload) 구성
            payload = {
                "path": path_coords
            }
            # self.get_logger().info(
            #     "\n" + "="*50 +
            #     f"\n🌐 [웹 전송 시작] Global Path 데이터 전송" +
            #     f"\n{json.dumps(preview_data, indent=2, ensure_ascii=False)}" +
            #     "\n" + "="*50
            # )

            # 4. Flask 서버로 전송 
            # 주의: 엔드포인트(/api/navigation/path)는 너희 웹 서버 API 주소에 맞게 수정해!
            url = f"{self.flask_base_url}/api/navigation/path" 
            headers = {"Content-Type": "application/json"}
            
            # 데이터 크기가 꽤 될 수 있으니 timeout을 2.0초로 넉넉하게 줬음
            res = requests.post(url, data=json.dumps(payload), headers=headers, timeout=2.0)
            
            # 5. 결과 확인
            if res.status_code == 200:
                self.get_logger().info(f"✅ Flask에 전체 경로 데이터 전송 성공! (좌표 {len(path_coords)}개)")
            else:
                self.get_logger().error(f"❌ 경로 데이터 전송 실패 (HTTP 상태 코드: {res.status_code})")
                
        except Exception as e:
            self.get_logger().error(f"❌ 웹으로 경로 전송 중 예외 발생: {e}")

    def _path_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2가 Path 계산 요청을 거절했습니다.")
            return

        self.get_logger().info("✅ Nav2가 Path 계산을 수락했습니다! 결과 도출 대기 중...")
        # 수락되었으니 최종 결과를 달라고 다시 요청하고 콜백 연결
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._path_get_result_callback)

    def _path_get_result_callback(self, future):
        result = future.result().result
        path = result.path  # 여기가 우리가 원하던 nav_msgs/Path 데이터야!
        
        # 경로가 안 나왔을 경우 방어 코드
        if not path.poses:
            self.get_logger().error("❌ Path 계산 실패: 반환된 경로 좌표가 없습니다.")
            return

        # 성공 로그 및 쪼개서 출력해 보기
        self.get_logger().info(f"🎉 [Path 획득 성공] 총 {len(path.poses)}개의 세부 궤적(Poses) 생성됨")
        
        # 데이터가 수백 개일 수 있으니, 터미널 테러 방지용으로 앞의 5개만 샘플로 찍어봄
        self.get_logger().info("🔎 [미리보기] 계산된 경로의 첫 5개 좌표: ")
        for i, pose_stamped in enumerate(path.poses[:5]):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.get_logger().info(f"   -> Pose {i+1}: x={x:.3f}, y={y:.3f}")
        self.get_logger().info(f" {type(path)}  -> ...) ...")
        ### 웹으로 보내는 함수 호출 !!
        self._send_path_to_flask(path)

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