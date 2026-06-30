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
        # 팩트: 갈 곳이 없거나, 행동 트리가 아직 Nav2에 목표를 쏘기도 전(goal_sent == False)이라면 거리 연산을 하지 않고 대기합니다.
        if not self.blackboard.has_goal or not self.blackboard.goal_sent:
            return

        # 1. 현재 로봇 위치와 목적지 사이의 유클리드 물리 거리 계산
        dx = self.blackboard.goal_x - self.blackboard.current_x
        dy = self.blackboard.goal_y - self.blackboard.current_y
        distance = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(f"🔍 [디버그] 목표: {self.blackboard.goal_name}, 남은거리: {distance:.3f}m", throttle_duration_sec=1.0)

        # 허용 반경 임계값 설정 (0.9m)
        arrival_threshold = 1.3

        # 2. 최초 도착 판단
        # 팩트: 아직 도착 플래그가 안 켜졌고 반경 이내로 들어왔다면 즉시 도착 스위치를 올리고 타이머를 가동합니다.
        if distance <= arrival_threshold and not self.blackboard.is_arrived:
            self.get_logger().info(f"🎯 목적지 [{self.blackboard.goal_name}] 근접 감지! 5초 대기를 시작합니다. (남은거리: {distance:.2f}m)")
            
            self.blackboard.is_arrived = True
            self.blackboard.wait_started = True
            self.blackboard.wait_start_time = time.time()

        # 3. 5초 대기 감시 및 다음 경유지 데이터 연산 전담 처리
        if self.blackboard.is_arrived and self.blackboard.wait_started:
            elapsed = time.time() - self.blackboard.wait_start_time
            self.get_logger().info(f"📍 [{self.blackboard.goal_name} 대기 중] 경과 시간: {elapsed:.1f}초", throttle_duration_sec=1.0)
            
            if elapsed >= 5.0:
                # 시나리오 A: 다음 남은 경유지가 리스트에 존재하는 경우
                if len(self.blackboard.waypoint_list) > 0:
                    next_wp = self.blackboard.waypoint_list.pop(0)
                    
                    # 블랙보드의 목적지 좌표 데이터를 다음 타겟으로 교체장전합니다.
                    self.blackboard.goal_name = next_wp["name"]
                    self.blackboard.goal_x = next_wp["x"]
                    self.blackboard.goal_y = next_wp["y"]
                    
                    # [핵심] 플래그 초기화
                    # 도착 플래그와 대기 플래그를 꺼서 상위 5번 브랜치(정지 제어)를 닫아버립니다.
                    self.blackboard.is_arrived = False
                    self.blackboard.wait_started = False
                    
                    # [자물쇠 해제] goal_sent 주행 잠금 플래그를 False로 열어줍니다.
                    # 이 스위치가 열리는 순간, 다음 틱에 행동 트리의 6번 브랜치가 새 목적지를 Nav2로 딱 한 번 발사하게 됩니다.
                    self.blackboard.goal_sent = False   
                    self.get_logger().info(f"➡️ 대기 완료. 다음 경유지 장전 및 주행 잠금 해제: {self.blackboard.goal_name}")
                
                # 시나리오 B: 리스트에 남은 경유지가 없는 최종 목적지인 경우
                else:
                    self.get_logger().info("✅ 모든 안내 여정이 최종 완료되었습니다.")
                    self.blackboard.has_goal = False
                    self.blackboard.is_arrived = False
                    self.blackboard.wait_started = False
                    self.blackboard.goal_sent = False


# ==============================================================================
# [DEBUG & LAUNCH INTERFACE] 단독 테스트 및 ROS 2 시스템 통합용 메인 엔트리 블록
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    
    # 🎯 팩트: 타 노드 종속성 없이 단독 실행(`python3 arrival_node.py`)할 때 
    # 참조 에러(AttributeError)가 발생하는 것을 막기 위해 최소한의 더미 가상 컨테이너를 주입합니다.
    class DummyBlackboard:
        def __init__(self):
            # 주행 타겟 기저 데이터 세팅
            self.has_goal = True
            self.goal_sent = True
            self.goal_name = "Debug_Station_A"
            self.goal_x = 1.5
            self.goal_y = 2.5
            
            # 오도메트리 수신용 기본 변수
            self.current_x = 0.0
            self.current_y = 0.0
            
            # 하드웨어 무결성 타이머 스위치
            self.last_sensor_time = time.time()
            self.sensor_timeout = False
            
            # 스케줄러 상태 스위치
            self.is_arrived = False
            self.wait_started = False
            self.wait_start_time = 0.0
            
            # 다음 가상의 경유지 데이터 적재 리스트 예시
            self.waypoint_list = [
                {"name": "Debug_Station_B", "x": -0.5, "y": 1.0}
            ]
            
    db = DummyBlackboard()
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