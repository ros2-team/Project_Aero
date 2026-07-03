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
import math

class WebBridgeNode(Node):
    def __init__(self, blackboard):
        super().__init__("web_bridge_node")
        self.blackboard = blackboard 
        ### path값 다 받아오기
        self.path_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.flask_base_url = "http://192.168.0.9:5000" 
        self.last_handled_command_id = -1             
        self.polling_interval = 0.5                   
        
        # 추가-> 이전 상태를 기억하여 변경 감지용 버퍼 변수 선언
        self.last_goal_state = None

        ### segment 저장용 리스트 선언
        self.segment_paths = []

        ########################### pub과 sub 선언 ############################
        self.web_command_pub = self.create_publisher(String, "/web/command", 10)
        self.bt_status_sub = self.create_subscription(
            String, "/robot/bt_status", self._bt_status_callback, 10
        )
        self.get_logger().info("확정 API 기반 ROS2 Web Bridge Node 가동 시작.")
        
        ############################# 백그라운드에서 실행시킬 스레드 생성 command_polling_loop 함수 콜백 #############################
        self.polling_thread = threading.Thread(target=self._command_polling_loop, daemon=True)
        self.polling_thread.start()

    ################################## 모든 기능 총 집합 함수 ################################
    def _command_polling_loop(self):
        """설정된 주기마다 Flask 서버를 폴링하며 새로운 명령이 생성되었는지 감시하는 루프"""
        while rclpy.ok():
            #################################### robot_status 데이터를 저장소 #################################
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

                            ################################### 웹으로부터 입력받은 목적지 좌표 저장소 ################################
                            raw_route = command_data.get("route", [])
                            processed_route = []

                            # 우리가 관리하는 전체 타깃 리스트 명시 (화장실, 면세점 부활!)
                            target_locations = ["게이트 A", "게이트 B", "게이트 C", "화장실", "면세점"]
                            
                            GATE_A = "게이트 A"
                            GATE_B = "게이트 B"
                            GATE_C = "게이트 C"

                            route_length = len(raw_route)
                            ################################# 
                            for i in range(route_length):
                                curr_wp = raw_route[i]
                                curr_name = curr_wp.get("location_name")
                                
                                # 1) 원본 경유지를 정제 리스트에 먼저 적재
                                processed_route.append(curr_wp)
                                
                                # 2) 마지막 목적지가 아니라면 '현재'와 '다음' 목적지 사이의 관계를 파악하여 경유지 주입
                                if i < route_length - 1:
                                    next_wp = raw_route[i+1]
                                    next_name = next_wp.get("location_name")

                                    # [안전 장치] 현재나 다음 목적지 중 하나라도 우리가 아는 타깃일 때만 우회 좌표 로직 실행
                                    if (curr_name in target_locations) or (next_name in target_locations):

                                        # [조건 1] A <-> B 구간 : 일직선이므로 경유지 없이 스킵
                                        if (curr_name == GATE_A and next_name == GATE_B) or \
                                           (curr_name == GATE_B and next_name == GATE_A):
                                            self.get_logger().info(f"⏩ [Route Planner] '{curr_name}' <-> '{next_name}' 일직선 구간. 경유지 없이 직진합니다.")
                                            continue

                                        # [조건 2] A <-> C 또는 B <-> C 구간 : 새로운 특별 우회 좌표(0.6, 0.0) 주입
                                        elif (curr_name in [GATE_A, GATE_B] and next_name == GATE_C) or \
                                             (curr_name == GATE_C and next_name in [GATE_A, GATE_B]):
                                            special_bypass_wp = {
                                                "location_name": "Special_Bypass_Point",
                                                "x": 0.6,
                                                "y": 0.0,
                                                "is_mid_point": True
                                            }
                                            processed_route.append(special_bypass_wp)
                                            self.get_logger().info(f"🔄 [Route Planner] '{curr_name}' <-> '{next_name}' 감지 -> 특별 우회 좌표(0.6, 0.0) 주입.")

                                        # [조건 3] 화장실, 면세점 등이 포함된 나머지 모든 루틴 : 기본 우회 좌표(0.0, 0.0) 주입
                                        else:
                                            default_mid_wp = {
                                                "location_name": "Default_Mid_Point",
                                                "x": 0.0,
                                                "y": 0.0,
                                                "is_mid_point": True
                                            }
                                            processed_route.append(default_mid_wp)
                                            self.get_logger().info(f"🔄 [Route Planner] '{curr_name}' <-> '{next_name}' 감지 -> 기본 우회 좌표(0.0, 0.0) 주입.")
                            # -------------------------------------------------------------------------
                            # 🎯 [Route Planner 레이어] 지정된 5개 목적지 직후 우회 좌표 동적 주입

                            # -------------------------------------------------------------------------
                            # raw_route = command_data.get("route", [])
                            # processed_route = []

                            # # 다중 코너 보정 타깃 리스트 정의
                            # target_locations = ["게이트 A", "게이트 B", "게이트 C", "화장실", "면세점"]

                            # for wp in raw_route:
                            #     # 1) 원본 경유지를 정제 리스트에 순서대로 먼저 적재
                            #     processed_route.append(wp)
                                
                            #     # 2) 수신된 목적지가 타깃 목록에 포함되어 있는지 검사
                            #     if wp.get("location_name") in target_locations:
                            #         mid_wp = {
                            #             "location_name": "Corner_Mid_Point",
                            #             "x": 0.1,   # 현장 환경에 맞춰 수정한 코너링 회피 X 좌표
                            #             "y": 0.1,  # 현장 환경에 맞춰 수정한 코너링 회피 Y 좌표
                            #             "is_mid_point": True  # ArrivalNode에서 무정차 식별용 플래그
                            #         }
                            #         processed_route.append(mid_wp)
                            #         self.get_logger().info(f"🔄 [Route Planner] '{wp.get('location_name')}' 감지 -> 코너링 우회용 중간 좌표를 경로 시퀀스에 강제 주입했습니다.")
                                    
                            ### 완성된 경로를 Nav2에 보내서 선(Path)으로 그려달라고 요청하기
                            poses_for_nav2 = []
                            for p_wp in processed_route:
                                pose = PoseStamped()
                                pose.header.frame_id = 'map'
                                pose.header.stamp = self.get_clock().now().to_msg()
                                pose.pose.position.x = float(p_wp.get("x", 0.0))
                                pose.pose.position.y = float(p_wp.get("y", 0.0))
                                poses_for_nav2.append(pose)
                            ### 예외처리
                            if not self.path_client.wait_for_server(timeout_sec=2.0):
                                self.get_logger().error("❌ Nav2 ComputePathThroughPoses 서버 응답 없음! (로봇 켜져있음?)")

                            else:
                                goal_msg = ComputePathThroughPoses.Goal()
                                goal_msg.goals = poses_for_nav2
                                self.get_logger().info(f"파일이 잘 담겼습니다 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1 {goal_msg.goals}")
                                
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
        print("웹에서 보내는 좌표입니다 !!!!!!!!!!!!!!!!", msg)
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

    def _send_path_to_flask(self, full_path, first_segment):
        """Nav2에서 연산된 전체 Global Path를 수신하여 Flask 서버로 HTTP POST 전송"""
        try:
            # 1. ROS Path 메시지에서 x, y 좌표만 빼서 리스트로 묶기
            path_coords = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in full_path.poses]
            
            # 2. 웹 서버로 보낼 JSON 페이로드(Payload) 구성
            payload = {
                "path": path_coords,
                "segments": [
                    self.segment_paths[0]
                ]
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
        # self.get_logger().info(f" {type(path)}  -> ...) ...")
        ### 웹으로 보내는 함수 호출 !!
        # Segment 생성
        self.segment_paths = self._build_segment_paths(path)

        # 최초 전체 Path + 첫 Segment 전송
        self._send_path_to_flask(
            full_path=path,
            first_segment=self.segment_paths[0]
)
        
    ############################## Nav2가 계산한 하나의 긴 Global Path를목적지 단위 Segment로 분리한다. ####################
    def _build_segment_paths(self, global_path):

        waypoints = self.blackboard.web_route_list

        # 목적지(중간 waypoint 제외) index만 추출
        target_indices = []

        for i, wp in enumerate(waypoints):
            if not wp.get("is_mid_point", False):
                target_indices.append(i)

        # 각 목적지가 global path의 몇 번째 pose인지 찾기
        pose_indices = []

        for idx in target_indices:

            wx = float(waypoints[idx]["x"])
            wy = float(waypoints[idx]["y"])

            best_idx = 0
            best_dist = float("inf")

            for i, pose in enumerate(global_path.poses):

                px = pose.pose.position.x
                py = pose.pose.position.y

                dist = (px-wx)**2 + (py-wy)**2

                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            pose_indices.append(best_idx)

        segments = []

        prev_pose_idx = 0

        for order in range(len(target_indices)):

            end_pose_idx = pose_indices[order]

            from_name = "current" if order == 0 else waypoints[target_indices[order-1]]["location_name"]

            to_name = waypoints[target_indices[order]]["location_name"]

            coords = []

            for pose in global_path.poses[prev_pose_idx:end_pose_idx+1]:

                coords.append({
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y
                })

            segments.append({
                "order": order,
                "from": from_name,
                "to": to_name,
                "path": coords
            })

            prev_pose_idx = end_pose_idx

        return segments
    
    def send_next_segment(self, order):

        if order >= len(self.segment_paths):
            return

        payload = {
            "path": [],
            "segments": [
                self.segment_paths[order]
            ]
        }

        try:

            url = f"{self.flask_base_url}/api/navigation/path"

            headers = {
                "Content-Type": "application/json"
            }

            requests.post(
                url,
                data=json.dumps(payload),
                headers=headers,
                timeout=2.0
            )

            self.get_logger().info(
                f"Segment {order} 전송 완료"
            )

        except Exception as e:

            self.get_logger().error(str(e))

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