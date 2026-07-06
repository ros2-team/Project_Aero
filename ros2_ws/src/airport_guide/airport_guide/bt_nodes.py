#!/usr/bin/env python3
import rclpy
from airport_guide.blackboard import GoalState, ChargingState

class BTNode:
    def __init__(self, name):
        self.name = name

    def tick(self, blackboard, ros_node):
        raise NotImplementedError

class Selector(BTNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def tick(self, blackboard, ros_node):
        for child in self.children:
            status = child.tick(blackboard, ros_node)
            if status != "FAILURE":
                return status
        return "FAILURE"

class Sequence(BTNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def tick(self, blackboard, ros_node):
        for child in self.children:
            status = child.tick(blackboard, ros_node)
            if status != "SUCCESS":
                return status
        return "SUCCESS"

# 0. 배터리 브랜치
class ConditionBatteryLow(BTNode):
    def tick(self, blackboard, ros_node):
        return "SUCCESS" if blackboard.battery_low else "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.charging_state == ChargingState.IDLE:
            ros_node.get_logger().error("🔋 배터리 부족 감지 -> 충전소 이동 시작")
            ros_node.cancel_nav_goal()
            ros_node.send_nav_goal(-0.07, -0.5)
            blackboard.charging_state = ChargingState.MOVING
        return "RUNNING"

# 1. 센서 감시 브랜치
class ConditionSensorTimeout(BTNode):
    def tick(self, blackboard, ros_node):
        return "SUCCESS" if blackboard.sensor_timeout else "FAILURE"

class ActionSensorEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("⚠️ [CRITICAL] 센서 데이터 유실 상태. 주행 정지 유도.", throttle_duration_sec=2.0)
        ros_node.cancel_nav_goal()
        return "RUNNING"

# 1.5. 일시정지 브랜치 노드
class ConditionWebPause(BTNode):
    def tick(self, blackboard, ros_node):
        # web_pause_node가 전처리해서 넣어준 플래그를 읽어서 판단
        return "SUCCESS" if blackboard.is_paused else "FAILURE"

class ActionWebPauseStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn("[PAUSE]시스템 일시정지", throttle_duration_sec=3.0)
        # 실제 물리 제어(Nav2 목표 취소)는 ros_node의 메서드를 호출하여 처리 (로봇 속도 알아서 0으로 떨어짐)
        ros_node.cancel_nav_goal()

        # web_pause 에서 goal_name의 ""를 지우면
        if blackboard.goal_name == "":
            ros_node.set_goal_state(GoalState.IDLE)
            blackboard.is_paused = False  # 초기화 완료 후 다음 주행을 위해
            return "SUCCESS"
        return "RUNNING"

# 2. 전방 충돌 방지 브랜치
class ConditionEmergency(BTNode):
    # 이제 이 노드는 순수하게 '지금 위험한가?'만 판단한다.
    # 위험 해제 후의 복구(재출발 준비) 로직은 별도 Action 노드로 분리했다.
    # Condition은 상태를 절대 바꾸지 않고 SUCCESS/FAILURE만 보고한다.
    def tick(self, blackboard, ros_node):
        if blackboard.is_dynamic_obstacle and blackboard.front_obstacle_distance <= 1.2:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("[🚨 EMERGENCY] 전방 충돌 위험권 진입. 즉시 제동 요청.", throttle_duration_sec=1.0)
        ros_node.cancel_nav_goal()
        return "RUNNING"

class ActionRecoverFromEmergency(BTNode):
    # [신규] 위험이 해제된 직후, FSM을 재출발 가능한 상태(IDLE)로 되돌리는 '복구 전용 Action'. 
    # 상태를 바꾸는 책임을 Condition에서 떼어내 이 노드 하나로 명시적으로 모았다. 
    # EmergencyBranch가 FAILURE를 반환해서 Root Selector가 다음 형제 브랜치로 넘어가기 전에, 
    # 이 브랜치가 먼저 복구 여부를 체크하고 지나가도록 EmergencyBranch 안에 배치한다.
    def tick(self, blackboard, ros_node):
        if blackboard.is_dynamic_obstacle:
            return "FAILURE"  # 아직 위험 → 복구할 필요 없음, 이 노드 통과 안 함
        
        if blackboard.goal_state in [GoalState.CANCELING, GoalState.IDLE] and blackboard.goal_name != "":
            ros_node.get_logger().info("🏃‍♂️ 전방 장애물 해제. FSM 복구 및 재출발 프로세스 시동.")
            # [핵심] 직접 대입(blackboard.goal_state = ...) 대신 ros_node에 위임.
            # 실제 enum 대입은 test_bt.py의 set_goal_state() 안에서만 일어난다.
            ros_node.set_goal_state(GoalState.IDLE)
            return "FAILURE"  # 복구는 부수 동작이므로 항상 FAILURE를 반환해 다음 브랜치로 넘어가게 함
        return "FAILURE"

# 3. 사용자 추적 브랜치
class ConditionHumanLost(BTNode):
    def tick(self, blackboard, ros_node):
        return "SUCCESS" if blackboard.human_lost else "FAILURE"

class ActionSearchHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("❓ [안내 유실] 대상 분실에 따른 제자리 정지.", throttle_duration_sec=3.0)
        ros_node.cancel_nav_goal()
        return "RUNNING"

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node):
        return "SUCCESS" if blackboard.human_far else "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn("📢 [대기] 가이드 대상 거리 이탈. 추격 대기 모드 진입.", throttle_duration_sec=3.0)
        ros_node.cancel_nav_goal()
        return "RUNNING"

# 4. 측후방 우회 브랜치
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.obstacle_warning or (0.0 < blackboard.rear_obstacle_distance <= 1.5):
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        return "RUNNING"

# 5. 경유지 도착 브랜치
class ConditionArrived(BTNode):
    def tick(self, blackboard, ros_node):
        return "SUCCESS" if blackboard.goal_state == GoalState.DONE else "FAILURE"

class ActionStopGuide(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.cancel_nav_goal()
        return "RUNNING"

# 6. 자율주행 최종 실행 브랜치 (수정본 반영 및 중복 제거)
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.goal_name != "" and blackboard.goal_state == GoalState.IDLE:
            return "SUCCESS"
        return "FAILURE"

class ActionMoveToGoal(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().info(f"🚀 [주행 제어] 액션 요청 송신 -> 타깃: {blackboard.goal_name}")
        # 🛠️ [핵심 수정] Nav2 목표를 보내기 직전에 상태를 RUNNING으로 변경
        # 이 변경으로 인해 다음 Tick부터 ConditionHasGoal 조건이 FAILURE가 되어 이 노드가 중복 호출되지 않습니다.
        blackboard.goal_state = GoalState.RUNNING
        # Nav2 액션 서버로 목표 전송 (내부적으로 SENT 상태 기록 등 수행)
        ros_node.send_nav_goal(blackboard.goal_x, blackboard.goal_y)
        return "RUNNING"

# 7. 기본 정적 대기 브랜치
class ActionIdle(BTNode):
    def tick(self, blackboard, ros_node):
        return "RUNNING"