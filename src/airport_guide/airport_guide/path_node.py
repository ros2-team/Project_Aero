#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class IndependentPathPreprocessor(Node):
    def __init__(self):
        super().__init__("path_preprocessor_node")
        
        # ROS2 서비스 호출 시 콜백 데드락 방지를 위한 그룹 설정 (이번엔 멍청하게 안 짰습니다 엣헴)
        self.cb_group = ReentrantCallbackGroup()

        # 1. 로봇 현재 위치 추적 (AMCL 맵핑 기준)
        self.current_x = 0.0
        self.current_y = 0.0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10, callback_group=self.cb_group)

        # 2. Nav2 플래너 서비스 클라이언트 (비동기 호출용)
        self.cli = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose',
            callback_group=self.cb_group
        )

        self.cli.wait_for_server()
        
        # 3. 통신 라인: 웹 명령 듣기(Sub) & 웹으로 경로 쏘기(Pub)
        self.create_subscription(String, '/web/command', self.command_callback, 10, callback_group=self.cb_group)
        self.path_pub = self.create_publisher(String, '/robot/full_path', 10)

        self.get_logger().info('🟢 독립형 경로 전처리 노드 가동 완료! (블랙보드 100% 격리)')

    def pose_callback(self, msg):
        """로봇의 실시간 현재 위치를 조용히 업데이트"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def command_callback(self, msg):
        """웹에서 새 주행 명령이 떨어지면 낚아채서 전처리 시작"""
        try:
            command_data = json.loads(msg.data)
            self.get_logger().info("일단 받았습니다 !!!!!!!!!!!!!!!!!!!!!!!!")
            # 주행 명령("navigation_route")일 때만 작동
            if command_data.get("action") == "navigation_route":
                route_list = command_data.get("payload", {}).get("route", [])
                
                if route_list:
                    self.get_logger().info(f"📥 목적지 {len(route_list)}개 수신! 궤적 전처리 시작...")
                    self.process_and_publish_path(route_list)
                    
        except Exception as e:
            self.get_logger().error(f"명령 파싱 에러: {e}")
            
    ######################### 웹에서 요구하는 메세지 타입으로 포장 작업 ##############################
    def process_and_publish_path(self, route_list):
        segments = []

        # 첫 출발점
        start_x = self.current_x
        start_y = self.current_y

        # 웹 표시용 segment 정보
        web_segment_path = []
        web_segment_order = 0
        web_segment_from = "current"

        for i, wp in enumerate(route_list):
            target_x = float(wp.get("x", 0.0))
            target_y = float(wp.get("y", 0.0))

            target_name = wp.get(
                "location_name",
                f"waypoint_{i}"
            )

            is_mid_point = wp.get(
                "is_mid_point",
                False
            )

            # 1. 현재 start -> wp까지 Nav2 path 계산
            segment_poses = self._request_nav2_path(
                start_x,
                start_y,
                target_x,
                target_y
            )

            partial_path = []

            for pose in segment_poses:
                px = round(pose.pose.position.x, 3)
                py = round(pose.pose.position.y, 3)

                partial_path.append({
                    "x": px,
                    "y": py
                })

            # 2. 현재 웹 segment path에 이어붙이기
            #    경계점 중복 방지를 위해 두 번째 구간부터는 첫 점 제거
            if web_segment_path and partial_path:
                web_segment_path.extend(
                    partial_path[1:]
                )
            else:
                web_segment_path.extend(
                    partial_path
                )

            # 3. 다음 구간 출발점 갱신
            start_x = target_x
            start_y = target_y

            # 4. 중간 경유지면 segment를 아직 publish하지 않음
            #    path만 누적하고 다음 wp로 넘어감
            if is_mid_point:
                continue

            # 5. 진짜 목적지에 도착하는 순간,
            #    지금까지 누적한 path를 하나의 웹 segment로 확정
            if len(web_segment_path) >= 2:
                segments.append({
                    "order": web_segment_order,
                    "from": web_segment_from,
                    "to": target_name,
                    "path": web_segment_path
                })

                self.get_logger().info(
                    f"📌 웹 segment 생성: order={web_segment_order}, "
                    f"from={web_segment_from}, to={target_name}, "
                    f"path_len={len(web_segment_path)}"
                )

            # 6. 다음 웹 segment 준비
            web_segment_order += 1
            web_segment_from = target_name
            web_segment_path = []

        navigation_path_state = {
            "segments": segments
        }

        result_msg = String()
        result_msg.data = json.dumps(
            navigation_path_state,
            ensure_ascii=False
        )

        self.path_pub.publish(result_msg)

        self.get_logger().info(
            f"🚀 웹 렌더링용 전체 경로 발행 완료! "
            f"(웹 segment {len(segments)}개)"
        )

    def _request_nav2_path(self, start_x, start_y, goal_x, goal_y):

        goal_msg = ComputePathToPose.Goal()

        goal_msg.start.header.frame_id = "map"
        goal_msg.start.pose.position.x = start_x
        goal_msg.start.pose.position.y = start_y
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.use_start = True

        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.pose.position.x = goal_x
        goal_msg.goal.pose.position.y = goal_y
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = "GridBased"

        # Goal 전송
        send_goal_future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Goal Handle 생성 실패")
            return []

        if not goal_handle.accepted:
            self.get_logger().error("Goal이 거부되었습니다.")
            return []

        # 결과 요청
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error("결과를 받지 못했습니다.")
            return []

        return result.result.path.poses

def main(args=None):
    rclpy.init(args=args)
    node = IndependentPathPreprocessor()
    
    # 🌟 독립 노드 전용 멀티스레드 실행기 (콜백 동시 처리용)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("종료 신호 수신. 안전하게 노드를 끕니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# import rclpy
# import json
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from std_msgs.msg import String
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav2_msgs.action import ComputePathToPose
# from rclpy.action import ActionClient
# import time

# class IndependentPathPreprocessor(Node):
#     def __init__(self):
#         super().__init__("path_preprocessor_node")
        
#         # ROS2 서비스 호출 시 콜백 데드락 방지를 위한 그룹 설정 (이번엔 멍청하게 안 짰습니다 엣헴)
#         self.cb_group = ReentrantCallbackGroup()

#         # 1. 로봇 현재 위치 추적 (AMCL 맵핑 기준)
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10, callback_group=self.cb_group)

#         # 2. Nav2 플래너 서비스 클라이언트 (비동기 호출용)
#         self.cli = ActionClient(
#             self,
#             ComputePathToPose,
#             '/compute_path_to_pose',
#             callback_group=self.cb_group
#         )

#         self.cli.wait_for_server()
        
#         # 3. 통신 라인: 웹 명령 듣기(Sub) & 웹으로 경로 쏘기(Pub)
#         self.create_subscription(String, '/web/command', self.command_callback, 10, callback_group=self.cb_group)
#         self.path_pub = self.create_publisher(String, '/robot/full_path', 10)

#         self.get_logger().info('🟢 독립형 경로 전처리 노드 가동 완료! (블랙보드 100% 격리)')

#     def pose_callback(self, msg):
#         """로봇의 실시간 현재 위치를 조용히 업데이트"""
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y

#     def command_callback(self, msg):
#         """웹에서 새 주행 명령이 떨어지면 낚아채서 전처리 시작"""
#         try:
#             command_data = json.loads(msg.data)
#             # self.get_logger().info("일단 받았습니다 !!!!!!!!!!!!!!!!!!!!!!!!")
#             # 주행 명령("navigation_route")일 때만 작동
#             if command_data.get("action") == "navigation_route":
#                 route_list = command_data.get("payload", {}).get("route", [])
                
#                 if route_list:
#                     self.get_logger().info(f"📥 목적지 {len(route_list)}개 수신! 궤적 전처리 시작...")
#                     self.process_and_publish_path(route_list)
                    
#         except Exception as e:
#             self.get_logger().error(f"명령 파싱 에러: {e}")

#     ######################## 260708 16:07 수정 코드 #########################
#     def process_and_publish_path(self, route_list):
        
#         segments = []
        
#         # 💡 [마술 도구 1] 웹에 표시할 '진짜' 출발지 이름 (경유지를 만나도 바뀌지 않음!)
#         real_start_name = "current" 
        
#         # 💡 [마술 도구 2] 여러 구간(경유지 포함)의 x, y 좌표를 끊기지 않게 계속 주워 담을 '임시 주머니'
#         accumulated_path = []

#         start_x = self.current_x
#         start_y = self.current_y

#         for i, wp in enumerate(route_list):
#             target_x = wp.get("x", 0.0)
#             target_y = wp.get("y", 0.0)
            
#             # DB에서 받은 진짜 이름 (예: 게이트 A, Corner_Right_Mid 등)
#             target_name = wp.get("location_name", f"waypoint_{i}")
            
#             # 🕵️‍♂️ [핵심] 이 목적지가 진짜 목적지인가? 아니면 몰래 숨겨야 할 '가짜 경유지'인가?
#             # web_bridge_node에서 넣어준 is_mid_point 값이 있거나, 이름에 Corner/MID가 들어가면 가짜로 판정!
#             is_mid_point = wp.get("is_mid_point", False) or "Corner" in target_name or "MID" in target_name

#             # 1. Nav2 플래너에게 현재 구간(start -> target) 궤적 요청
#             segment_poses = self._request_nav2_path(start_x, start_y, target_x, target_y)

#             # 2. 받아온 궤적을 임시 주머니(accumulated_path)에 무지성으로 계속 쏟아붓기 (+= 개념)
#             for pose in segment_poses:
#                 px = round(pose.pose.position.x, 3)
#                 py = round(pose.pose.position.y, 3)
#                 accumulated_path.append({"x": px, "y": py})

#             # 🎁 3. 드디어 '진짜 목적지'를 만났을 때만 짐(segment)을 쌉니다!
#             if not is_mid_point:
#                 segments.append({
#                     "order": wp.get("order", i),
#                     "from": real_start_name,
#                     "to": target_name,
#                     "path": accumulated_path  # 그동안 주머니에 모아둔 꺾어지는 궤적 좌표를 몽땅 털어 넣음!
#                 })
                
#                 # 짐을 다 쌌으니 다음 진짜 목적지를 위해 세팅 리셋!
#                 accumulated_path = []          # 주머니 비우기
#                 real_start_name = target_name  # 다음 구간의 웹 표시용 출발지는 방금 도착한 '이 진짜 목적지'가 됨

#             # 4. 다음 반복을 위해 Nav2 출발지 좌표는 무조건 갱신 (경유지든 진짜 목적지든 꼬리물기는 계속되어야 함)
#             start_x = target_x
#             start_y = target_y

#         # 5. 프론트엔드 명세서와 100% 일치하는 최종 딕셔너리 조립
#         navigation_path_state = {
#             "segments": segments
#         }

#         # 6. JSON으로 묶어서 토픽 발행
#         result_msg = String()
#         result_msg.data = json.dumps(navigation_path_state)
#         self.path_pub.publish(result_msg)
#         self.get_logger().info(f"🚀 웹 렌더링용 경로 토픽 발행 완료! (최종 압축된 구간 {len(segments)}개)")
#     # ######################### 웹에서 요구하는 메세지 타입으로 포장 작업 ##############################
#     # def process_and_publish_path(self, route_list):
        
#     #     ##### segment를 담는다 #######
#     #     segments = []
        
#     #     # 첫 출발점 세팅
#     #     start_x = self.current_x
#     #     start_y = self.current_y
#     #     prev_name = "current" # 첫 출발지 이름은 'current'

#     #     for i, wp in enumerate(route_list):
#     #         target_x = wp.get("x", 0.0)
#     #         target_y = wp.get("y", 0.0)
#     #         # 목적지 이름 (DB에서 받은 이름 그대로 사용)
#     #         target_name = wp.get("location_name", f"waypoint_{i}")
#     #         current_order = i

#     #         # 1. Nav2 플래너에게 구간 궤적 요청
#     #         segment_poses = self._request_nav2_path(start_x, start_y, target_x, target_y)

#     #         # 2. 받아온 궤적을 명세서 양식 {"x": 1.2, "y": 3.4} 형태로 가공
#     #         segment_path = []
#     #         for pose in segment_poses:
#     #             px = round(pose.pose.position.x, 3)
#     #             py = round(pose.pose.position.y, 3)
#     #             coord = {"x": px, "y": py}


#     #             ########## x,y값을 담는다. #########
#     #             segment_path.append(coord)

#     #         # 3. segments 배열에 현재 구간 정보 쏙 집어넣기
#     #         segments.append({
#     #             "order": current_order,
#     #             "from": prev_name,
#     #             "to": target_name,
#     #             "path": segment_path
#     #         })

#     #         # 4. 다음 구간을 위해 출발지 갱신 (꼬리물기)
#     #         start_x = target_x
#     #         start_y = target_y
#     #         prev_name = target_name

#     #     # 5. 프론트엔드 명세서와 100% 일치하는 최종 딕셔너리 조립
#     #     navigation_path_state = {
#     #         "segments": segments
#     #     }

#     #     # 6. JSON으로 묶어서 토픽 발행
#     #     result_msg = String()
#     #     result_msg.data = json.dumps(navigation_path_state)
#     #     self.path_pub.publish(result_msg)
#     #     self.get_logger().info(f"🚀 웹 렌더링용 경로 토픽 발행 완료! (구간 {len(segments)}개)")
#         # self.get_logger().info(f"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@{result_msg.data}")

#     def _request_nav2_path(self, start_x, start_y, goal_x, goal_y):

#         goal_msg = ComputePathToPose.Goal()

#         goal_msg.start.header.frame_id = "map"
#         goal_msg.start.pose.position.x = start_x
#         goal_msg.start.pose.position.y = start_y
#         goal_msg.start.pose.orientation.w = 1.0
#         goal_msg.use_start = True

#         goal_msg.goal.header.frame_id = "map"
#         goal_msg.goal.pose.position.x = goal_x
#         goal_msg.goal.pose.position.y = goal_y
#         goal_msg.goal.pose.orientation.w = 1.0
#         goal_msg.planner_id = "GridBased"

#         # Goal 전송
#         send_goal_future = self.cli.send_goal_async(goal_msg)
        
#         # 🚀 1. spin 대신 미래(future)가 끝날 때까지 0.05초씩 쉬면서 조용히 기다립니다.
#         while rclpy.ok() and not send_goal_future.done():
#             time.sleep(0.05) 

#         goal_handle = send_goal_future.result()

#         if goal_handle is None:
#             self.get_logger().error("Goal Handle 생성 실패")
#             return []

#         if not goal_handle.accepted:
#             self.get_logger().error("Goal이 거부되었습니다.")
#             return []

#         # 결과 요청
#         result_future = goal_handle.get_result_async()
        
#         # 🚀 2. 여기서도 spin 대신 반복문으로 기다립니다.
#         while rclpy.ok() and not result_future.done():
#             time.sleep(0.05)
            
#         # # Goal 전송
#         # send_goal_future = self.cli.send_goal_async(goal_msg)
#         # rclpy.spin_until_future_complete(self, send_goal_future)

#         # goal_handle = send_goal_future.result()

#         # if goal_handle is None:
#         #     self.get_logger().error("Goal Handle 생성 실패")
#         #     return []

#         # if not goal_handle.accepted:
#         #     self.get_logger().error("Goal이 거부되었습니다.")
#         #     return []

#         # # 결과 요청
#         # result_future = goal_handle.get_result_async()
#         # rclpy.spin_until_future_complete(self, result_future)

#         result = result_future.result()

#         if result is None:
#             self.get_logger().error("결과를 받지 못했습니다.")
#             return []

#         return result.result.path.poses

# def main(args=None):
#     rclpy.init(args=args)
#     node = IndependentPathPreprocessor()
    
#     # 🌟 독립 노드 전용 멀티스레드 실행기 (콜백 동시 처리용)
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
    
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         node.get_logger().info("종료 신호 수신. 안전하게 노드를 끕니다.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()