#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time
from rclpy.qos import qos_profile_sensor_data

class ArrivalNode(Node):
    def __init__(self, blackboard):
        super().__init__('arrival_node')
        self.blackboard = blackboard
 
        # 로봇의 현재 위치를 알기 위해 Odometry 토픽을 구독합니다. (QoS 최적화)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        # 도착 여부 및 타이머 처리를 주기적으로 체크하기 위한 타이머 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.check_arrival)
        
        self.get_logger().info("✅ Arrival Node가 성공적으로 시작되었습니다.")

    def odom_callback(self, msg):
        # 로봇의 현재 x, y 좌표 기록 및 센서 타임아웃 방지 플래그 갱신
        self.blackboard.current_x = msg.pose.pose.position.x
        self.blackboard.current_y = msg.pose.pose.position.y
        self.blackboard.last_sensor_time = time.time()
        self.blackboard.sensor_timeout = False

    def check_arrival(self):
        # 가야 할 목적지가 없다면 연산을 수행하지 않고 패스합니다.
        if not self.blackboard.has_goal:
            return

        # 💡 [에러 해결 및 방어막 코드]
        # 행동 트리가 다음 경유지 좌표를 바꾸고 상태를 "IDLE"로 세팅한 순간에는 거리를 계산하지 않습니다.
        # 행동 트리가 새 목표를 Nav2로 완전히 쏘고 상태를 "EXECUTING"으로 복구했을 때만 연산을 재개합니다.
        if self.blackboard.nav_status != "EXECUTING":
            return

        # 1. 거리 계산
        dx = self.blackboard.goal_x - self.blackboard.current_x
        dy = self.blackboard.goal_y - self.blackboard.current_y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"🔍 [디버그] 목표: {self.blackboard.goal_name}, 남은거리: {distance:.3f}m, 상태: {self.blackboard.nav_status}", throttle_duration_sec=0.5)

        arrival_threshold = 0.9

        # 2. 최초 도착 판단
        if self.blackboard.nav_status == "EXECUTING" and distance <= arrival_threshold and not self.blackboard.is_arrived:
            self.get_logger().info(f"🎯 목적지 [{self.blackboard.goal_name}] 근접 감지! 5초 대기를 시작합니다. (남은거리: {distance:.2f}m)")
            
            self.blackboard.is_arrived = True
            self.blackboard.wait_started = True
            self.blackboard.wait_start_time = time.time()

        # 3. 5초 대기 감시 및 다음 경유지 데이터 연산 전담 처리
        if self.blackboard.is_arrived and self.blackboard.wait_started:
            elapsed = time.time() - self.blackboard.wait_start_time
            self.get_logger().info(f"📍 [{self.blackboard.goal_name} 대기 중] 경과 시간: {elapsed:.1f}초", throttle_duration_sec=1.0)
            
            if elapsed >= 5.0:
                # 다음 경유지가 남아 있는 경우
                if len(self.blackboard.waypoint_list) > 0:
                    next_wp = self.blackboard.waypoint_list.pop(0)
                    # 리스트 맨 앞 꺼내서 목적지 교체
                    self.blackboard.goal_name = next_wp["name"]
                    self.blackboard.goal_x = next_wp["x"]
                    self.blackboard.goal_y = next_wp["y"]
                    
                    # 상태 초기화하여 다시 출발하도록 설정
                    self.blackboard.is_arrived = False
                    self.blackboard.wait_started = False
                    
                    # 💡 중요: 이 노드가 IDLE로 바꾸는 즉시 위쪽의 방어막에 걸려
                    # 트리가 새 목표를 쏘고 다시 EXECUTING으로 만들 때까지 거리 계산 루프가 안전하게 멈춥니다.
                    self.blackboard.nav_status = "IDLE"   
                    self.get_logger().info(f"➡️ 대기 완료. 다음 경유지 장전: {self.blackboard.goal_name}")
                
                # 리스트에 남은 경유지가 없는 최종 목적지인 경우
                else:
                    # 최종 목적지 도달 완료
                    self.get_logger().info("✅ 모든 안내 여정이 끝났습니다.")
                    self.blackboard.has_goal = False
                    self.blackboard.nav_status = "IDLE"
                    self.blackboard.is_arrived = False
                    self.blackboard.wait_started = False