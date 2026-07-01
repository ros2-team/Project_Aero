#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time
from rclpy.qos import qos_profile_sensor_data
# 🎯 리팩토링된 FSM 상태 ENUM 임포트
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
        
        # 내부 대기 세션 관리를 위한 지역 변수 (블랙보드 밖으로 격리)
        self.local_wait_started = False
        self.local_wait_start_time = 0.0

        self.get_logger().info("✅ [Data Layer] Arrival Node가 가동되었습니다.")

    def odom_callback(self, msg):
        # [FACTS AREA WRITE] 순수 센서 데이터만 업데이트
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.last_sensor_time = time.time()
        # DERIVED 레이어 상태 해제는 추후 Evaluator로 통합 가능하나 현 단계 유지
        self.blackboard.sensor_timeout = False 

    def check_arrival(self):
        # ---------------------------------------------------------------------
        # 🎯 [Data Layer] 웹 데이터 버퍼 처리 및 목적지 분기 판단 레이어
        # ---------------------------------------------------------------------
        # 1. 웹 브릿지가 준 action이 "navigation_route"일 때 (새 경로 최초 주입)
        if self.blackboard.web_action == "navigation_route":
            if self.blackboard.web_route_list:
                next_wp = self.blackboard.web_route_list.pop(0)
                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                
                self.blackboard.goal_state = GoalState.IDLE
                self.get_logger().info(f"🌐 [Web Route] 첫 번째 목적지 주입 완료: {self.blackboard.goal_name}")
                
            self.blackboard.web_action = ""
            return  # 데이터 세팅 직후 물리 연산 건너뛰고 다음 틱 대기

        # 2. 현재 경유지 정지/대기가 완료(DONE)되었을 때 (차기 목적지 토스)
        elif self.blackboard.goal_state == GoalState.DONE:
            if self.blackboard.web_route_list:
                next_wp = self.blackboard.web_route_list.pop(0)
                self.blackboard.goal_name = next_wp.get("location_name", "Unknown")
                self.blackboard.goal_x = float(next_wp.get("x", 0.0))
                self.blackboard.goal_y = float(next_wp.get("y", 0.0))
                
                self.blackboard.goal_state = GoalState.IDLE
                self.get_logger().info(f"🚗 [Route Relay] 다음 경유지로 이동합니다: {self.blackboard.goal_name}")
            else:
                self.blackboard.goal_name = ""
                self.blackboard.goal_state = GoalState.IDLE
                self.get_logger().info("🎉 [Route Completed] 모든 지정 경유지 주행이 최종 완료되었습니다.")
            
            return  # 데이터 전환 직후 물리 연산 건너뛰고 다음 틱 대기

        # ---------------------------------------------------------------------
        # 🎯 [Control Layer] 기존 ArrivalNode의 거리 측정 및 대기 로직
        # ---------------------------------------------------------------------
        # 행동트리가 자율주행을 시작하여 RUNNING 상태로 만들기 전까지는 거리 연산을 하지 않음
        if self.blackboard.goal_state != GoalState.RUNNING:
            self.local_wait_started = False
            return

        # 1. 물리 거리 연산
        dx = self.blackboard.goal_x - self.blackboard.current_x
        dy = self.blackboard.goal_y - self.blackboard.current_y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"🔍 [디버그] 목표: {self.blackboard.goal_name}, 남은거리: {distance:.3f}m", throttle_duration_sec=2.0)

        # 실내 자율주행 정석 반경 (0.9m)
        arrival_threshold = 0.9

        # 2. 최초 도착 판단 
        if distance <= arrival_threshold and not self.local_wait_started:
            self.get_logger().info(f"🎯 목적지 [{self.blackboard.goal_name}] 반경 진입. 5초 대기를 시작합니다.")
            self.local_wait_started = True
            self.local_wait_start_time = time.time()

        # 3. 5초 대기 감시 및 FSM 상태 전이
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