#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time
from rclpy.qos import qos_profile_sensor_data
# 🎯 FSM 상태 ENUM 임포트
from airport_guide.blackboard import GoalState 

class ArrivalNode(Node):
    def __init__(self, blackboard):
        super().__init__('arrival_node')
        self.blackboard = blackboard
 
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(0.1, self.check_arrival)
        
        # 내부 대기 세션 관리를 위한 지역 변수
        self.local_wait_started = False
        self.local_wait_start_time = 0.0
        
        # ****** 현재 목적지가 WebBridge에서 심어둔 중간 우회 좌표인지 식별할 플래그
        self.is_current_mid_point = False

        self.get_logger().info("✅ [Data Layer] Arrival Node가 가동되었습니다.")

    def odom_callback(self, msg):
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False 

    def check_arrival(self):
        # ---------------------------------------------------------------------
        # 웹에서 '종료(stop_navigation)' 명령이 들어왔을 때 리셋 처리
        # ---------------------------------------------------------------------
        if self.blackboard.web_action == "stop_navigation":
            self.get_logger().warn("🛑 [Web Command] 주행 종료 명령 수신. 모든 경로 데이터를 리셋합니다.")
            
            self.blackboard.goal_name = ""
            self.blackboard.goal_x = 0.0
            self.blackboard.goal_y = 0.0
            self.blackboard.web_route_list = []  
            self.blackboard.goal_state = GoalState.IDLE
            
            self.local_wait_started = False
            self.local_wait_start_time = 0.0
            self.is_current_mid_point = False
            self.blackboard.web_action = ""
            return

        # ---------------------------------------------------------------------
        # [Data Layer] 웹 데이터 버퍼 처리 및 목적지 분기 판단 레이어
        # ---------------------------------------------------------------------
        is_new_route_cmd = (self.blackboard.web_action == "navigation_route")
        is_system_idle_with_queue = (self.blackboard.goal_state == GoalState.IDLE and 
                                     self.blackboard.goal_name == "" and 
                                     bool(self.blackboard.web_route_list))

        # 처음 기동하거나 새로운 경로가 들어왔을 때 데이터 파싱
        if is_new_route_cmd or is_system_idle_with_queue:
            if self.blackboard.web_route_list:
                next_wp = self.blackboard.web_route_list.pop(0)
                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                
                # ****** 꺼내온 노드가 WebBridge에서 전처리한 중간 우회점인지 백업 확인
                self.is_current_mid_point = next_wp.get("is_mid_point", False)
                
                self.blackboard.goal_state = GoalState.IDLE
                self.get_logger().info(f"🌐 [Web Route] 목적지 주입 완료 ➔ 타깃: {self.blackboard.goal_name} (MidPoint: {self.is_current_mid_point})")
                
            self.blackboard.web_action = ""

        # 2. 현재 경유지 정지/대기가 완료(DONE)되었을 때 (차기 목적지 토스)
        if self.blackboard.goal_state == GoalState.DONE:
            if self.blackboard.web_route_list:
                next_wp = self.blackboard.web_route_list.pop(0)
                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                
                # 🎯 [추가] 릴레이 시점에도 동일하게 중간 노드 메타데이터 판별 스위치 온
                self.is_current_mid_point = next_wp.get("is_mid_point", False)
                
                self.blackboard.goal_state = GoalState.IDLE
                self.get_logger().info(f"🚗 [Route Relay] 다음 경유지로 이동합니다: {self.blackboard.goal_name} (MidPoint: {self.is_current_mid_point})")
            else:
                self.blackboard.goal_name = ""
                self.blackboard.goal_state = GoalState.IDLE
                self.is_current_mid_point = False
                self.get_logger().info("🎉 [Route Completed] 모든 지정 경유지 주행이 최종 완료되었습니다.")
                return

        # ---------------------------------------------------------------------
        # ****** [Control Layer] 거리 측정 및 대기 가드 조건
        # ---------------------------------------------------------------------
        if self.blackboard.goal_state != GoalState.RUNNING:
            if self.blackboard.goal_state != GoalState.IDLE:
                self.local_wait_started = False
            return  

        # 1. 물리 거리 연산
        dx = self.blackboard.goal_x - self.blackboard.current_x
        dy = self.blackboard.goal_y - self.blackboard.current_y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"🔍 [디버그] 목표: {self.blackboard.goal_name}, 남은거리: {distance:.3f}m", throttle_duration_sec=2.0)

        # ******중간 우회 경로인 경우, 로봇이 서지 않고 지나가도록 허용 허브 반경 조절
        if self.is_current_mid_point:
            arrival_threshold = 0.6  # 코너 영역은 스치듯 유연하게 도달 허용 범위를 넓힘
        else:
            arrival_threshold = 0.9  # 일반 정식 경유지는 정밀 도달

        # 2. 최초 도착 판단 
        if distance <= arrival_threshold and not self.local_wait_started:
            
            # ******만약 플래너가 심은 중간 우회 좌표라면 5초 멈추지 않고 즉시 패스스루 처리
            if self.is_current_mid_point:
                self.get_logger().info(f"🔄 [Corner Pass-through] 중간 우회지 [{self.blackboard.goal_name}] 통과. 정지 없이 릴레이 연계.")
                self.blackboard.goal_state = GoalState.DONE
                return

            self.get_logger().info(f"🎯 목적지 [{self.blackboard.goal_name}] 반경 진입. 5초 대기를 시작합니다.")
            self.local_wait_started = True
            self.local_wait_start_time = time.time()

        # 3. 5초 대기 감시 및 FSM 상태 전이 (일반 노드 전용)
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