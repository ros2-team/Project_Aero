#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String  # 🎯 토픽 구독을 위해 추가
import json                      # 🎯 JSON 파싱을 위해 추가
import math
import time
from rclpy.qos import qos_profile_sensor_data
from airport_guide.blackboard import GoalState
from geometry_msgs.msg import PoseWithCovarianceStamped

class ArrivalNode(Node):
    def __init__(self, blackboard):
        super().__init__('arrezzival_node')
        self.blackboard = blackboard
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     qos_profile_sensor_data
        # )

        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        # WebBridgeNode가 발행하는 웹 명령 토픽 직접 구독 추가
        self.command_sub = self.create_subscription(
            String,
            '/web/command',
            self.web_command_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.check_arrival)
        self.local_wait_started = False
        self.local_wait_start_time = 0.0
        self.is_current_mid_point = False
        self.get_logger().info("✅ [Data Layer] Arrival Node가 정상 가동 및 토픽 구독을 시작했습니다.")

    def web_command_callback(self, msg):
        try:
            raw_data = json.loads(msg.data)
            action_type = raw_data.get("action")  # web_bridge가 보낸 규격 매칭
            payload = raw_data.get("payload", {})   # raw_data.get("payload")로 이 덩어리를 가져옴
            route = payload.get("route", []) # 실제 루트 꺼내옴
            
            if action_type == "navigation_route":   # 안내시작 누르면 실행 됨
            
                self.blackboard.web_route_list = route
                self.blackboard.web_action = "navigation_route"
                self.blackboard.web_last_update_time = time.time()

                self.blackboard.current_waypoint_index = 0
                self.blackboard.navigation_active = True
                self.blackboard.navigation_finished = False
                
                self.blackboard.goal_name = ""
                self.blackboard.goal_x = 0.0
                self.blackboard.goal_y = 0.0
                self.blackboard.goal_state = GoalState.IDLE

                self.local_wait_started = False
                self.local_wait_start_time = 0.0
                self.is_current_mid_point = False
                
                self.get_logger().info(
                    f"[Arrival Node] 새로운 웹 경로 명령 감지 완료. 경유지 수: {len(route)}"
                )
            elif action_type == "stop_navigation":
                self.blackboard.web_action = "stop_navigation"

            # 웹에서 '주행 재개' 버튼을 눌렀을 때 처리 루틴
            elif action_type == "resume_navigation":
                # 현재 블랙보드에 목적지가 남아있는데 상태가 IDLE(또는 일시정지) 상태라면
                if self.blackboard.goal_name != "":
                    self.blackboard.goal_state = GoalState.RUNNING
                    self.get_logger().info(f"▶️ [Arrival Node] 주행 재개 명령 수신 -> FSM 상태를 RUNNING으로 강제 복구합니다.")
        except Exception as e:
            self.get_logger().error(f"❌ [Arrival Node] 웹 명령 파싱 오류: {e}")

    # def odom_callback(self, msg):
        # self.blackboard.current_x = msg.pose.pose.position.x
        # self.blackboard.current_y = msg.pose.pose.position.y
        # self.blackboard.last_sensor_time = time.time()
        # self.blackboard.sensor_timeout = False

    def pose_callback(self, msg):
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False

    # def odom_callback(self, msg):
    #     self.blackboard.current_x = msg.pose.pose.position.x
    #     self.blackboard.current_y = msg.pose.pose.position.y
    #     self.blackboard.last_sensor_time = time.time()
    #     self.blackboard.sensor_timeout = False

    def check_arrival(self):
        if self.blackboard.web_action == "stop_navigation":
            self.get_logger().warn("🛑 [Web Command] 주행 종료 명령 수신. 모든 경로 데이터를 리셋합니다.")
            
            self.blackboard.goal_name = ""
            self.blackboard.goal_x = 0.0
            self.blackboard.goal_y = 0.0

            self.blackboard.web_route_list = []
            self.blackboard.current_waypoint_index = 0
            self.blackboard.navigation_active = False
            self.blackboard.navigation_finished = False

            self.blackboard.goal_state = GoalState.IDLE
            
            self.local_wait_started = False
            self.local_wait_start_time = 0.0
            self.is_current_mid_point = False
            
            self.blackboard.web_action = ""
            return

        # [안내시작] 눌렀는가?
        is_new_route_cmd = (self.blackboard.web_action == "navigation_route" and
                            bool(self.blackboard.web_route_list) and
                            self.blackboard.goal_name == "") # 🛠️ 가드 추가

        # 대기상태 & 현재 경유지도 없음 & 다음 경유지가 있음
        is_system_idle_with_queue = (self.blackboard.goal_state == GoalState.IDLE and
                                     self.blackboard.goal_name == "" and
                                     bool(self.blackboard.web_route_list))
        
        if is_new_route_cmd or is_system_idle_with_queue:
            route = self.blackboard.web_route_list
            current_index = self.blackboard.current_waypoint_index
            
            self.get_logger().info(
                f"[DEBUG] 목적지 주입 조건 진입. "
                f"전체 경로 개수 : {len(route)}, 현재 index : {current_index}"
            )

            if current_index < len(route):
                next_wp = route[current_index]

                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                self.is_current_mid_point = next_wp.get("is_mid_point", False)

                self.blackboard.goal_state = GoalState.IDLE

                self.get_logger().info(
                    f"📥 목적지 주입 완료 ➔ "
                    f"[{current_index + 1}/{len(route)}] "
                    f"타깃: {self.blackboard.goal_name} "
                    f"(FSM: IDLE / BT 틱 대기)"
                )
            else:
                self.get_logger().info(
                    "[DEBUG] current_waypoint_index가 route 길이를 초과해서 목적지 주입 실패!"
                )
            
            self.blackboard.web_action = ""
            return
        if self.blackboard.goal_state == GoalState.DONE:
            route = self.blackboard.web_route_list

            self.blackboard.current_waypoint_index += 1
            current_index = self.blackboard.current_waypoint_index

            self.get_logger().info(
                f"✅ 경유지 도착 처리 완료. "
                f"다음 index: {current_index}, 전체 경로 수: {len(route)}"
            )

            if current_index < len(route):
                next_wp = route[current_index]

                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                self.is_current_mid_point = next_wp.get("is_mid_point", False)

                self.blackboard.goal_state = GoalState.IDLE
                self.blackboard.navigation_active = True
                self.blackboard.navigation_finished = False

                self.local_wait_started = False
                self.local_wait_start_time = 0.0

                self.get_logger().info(
                    f"➡️ 다음 경유지 이동 준비: "
                    f"[{current_index + 1}/{len(route)}] "
                    f"{self.blackboard.goal_name} "
                    f"(FSM: IDLE)"
                )
            else:
                self.blackboard.goal_name = ""
                self.blackboard.goal_x = 0.0
                self.blackboard.goal_y = 0.0

                self.blackboard.current_waypoint_index = len(route)
                self.blackboard.navigation_active = False
                self.blackboard.navigation_finished = True

                self.blackboard.goal_state = GoalState.IDLE
                self.is_current_mid_point = False

                self.local_wait_started = False
                self.local_wait_start_time = 0.0

                self.get_logger().info(
                    "🎉 모든 지정 경유지 주행이 최종 완료되었습니다."
                )

            return

        # ---------------------------------------------------------------------
        # [Control Layer] 제어 가드 조건 (완벽 방어벽)
        # ---------------------------------------------------------------------
        # 🛠️ 로봇이 실제 주행 중(RUNNING)이 아니라면 아래의 거리 계산/타이머 로직을 아예 '무시'하고 종료합니다.
        # IDLE 상태일 때 하단 거리 계산으로 흘러내려가던 치명적인 구멍을 차단합니다.
        if self.blackboard.goal_state != GoalState.RUNNING:
            self.local_wait_started = False
            return

        # 🏃‍♂️ 여기 아래부터는 오직 goal_state가 "RUNNING"일 때만 도달하여 실행됩니다.
        dx = self.blackboard.goal_x - self.blackboard.current_x
        dy = self.blackboard.goal_y - self.blackboard.current_y
        distance = math.sqrt(dx**2 + dy**2)  # 🛠️ 유효한 파이썬 거듭제곱 연산으로 수정
        self.get_logger().info(f"🔍 [디버그 주행 중] 목표: {self.blackboard.goal_name}, 남은거리: {distance:.3f}m", throttle_duration_sec=2.0)
        
        if self.is_current_mid_point:
            arrival_threshold = 0.3
        else:
            arrival_threshold = 0.4

        if distance <= arrival_threshold and not self.local_wait_started:
            if self.is_current_mid_point:
                self.get_logger().info(f"🔄 [Corner Pass-through] 중간 우회지 [{self.blackboard.goal_name}] 통과. 정지 없이 릴레이 연계.")
                self.blackboard.goal_state = GoalState.DONE
                return
            self.get_logger().info(f"🎯 목적지 [{self.blackboard.goal_name}] 반경 진입. 5초 대기를 시작합니다.")
            self.local_wait_started = True
            self.local_wait_start_time = time.time()

        if self.local_wait_started:
            elapsed = time.time() - self.local_wait_start_time
            self.get_logger().info(f"📍 [{self.blackboard.goal_name} 대기 중] 경과 시간: {elapsed:.1f}초", throttle_duration_sec=1.0)
            if elapsed >= 5.0:
                self.get_logger().info(f"✅ [{self.blackboard.goal_name}] 안내 세션 종료. FSM 상태를 DONE으로 전이합니다.")
                self.blackboard.goal_state = GoalState.DONE
                self.local_wait_started = False

def main(args=None):
    rclpy.init(args=args)
    from airport_guide.blackboard import Blackboard
    db = Blackboard()
    node = ArrivalNode(blackboard=db)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()