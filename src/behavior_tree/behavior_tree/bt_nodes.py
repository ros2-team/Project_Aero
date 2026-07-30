#!/usr/bin/env python3
import rclpy
from behavior_tree.blackboard import GoalState, ChargingState

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
        return "SUCCESS" if blackboard.battery_level < 35 else "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.charging_state == ChargingState.IDLE:
            ros_node.get_logger().error("🔋 배터리 부족 감지 -> 충전소 이동 시작")
            ros_node.cancel_nav_goal()
            ros_node.send_nav_goal(-0.029, -0.927)
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
        # 🛠️ [재개 판정 추가] 웹 콜백에 의해 일시정지 플래그가 해제된 경우
        if not blackboard.is_paused:
            ros_node.set_goal_state(GoalState.IDLE)  # 주행 가드(ConditionHasGoal)를 열어주기 위해 IDLE 전이
            return "SUCCESS"  # 일시정지 브랜치를 완전히 탈출

        # 여전히 일시정지 상태(True)일 때만 하부 제동 로직 수행
        ros_node.get_logger().warn("[PAUSE]시스템 일시정지 상태 (대기 중...)", throttle_duration_sec=3.0)
        ros_node.cancel_nav_goal()

        if blackboard.goal_name == "":
            ros_node.set_goal_state(GoalState.IDLE)
            blackboard.is_paused = False  
            return "SUCCESS"
        return "RUNNING"

# 2. 전방 충돌 방지 브랜치
class ConditionEmergency(BTNode):
    # 이제 이 노드는 순수하게 '지금 위험한가?'만 판단한다.
    # 위험 해제 후의 복구(재출발 준비) 로직은 별도 Action 노드로 분리했다.
    # Condition은 상태를 절대 바꾸지 않고 SUCCESS/FAILURE만 보고한다.
    def tick(self, blackboard, ros_node):
        if blackboard.front_obstacle_distance is not None and blackboard.front_obstacle_distance <= 80:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("[🚨 EMERGENCY] 전방 충돌 위험권 진입. 즉시 제동 요청.", throttle_duration_sec=1.0)
        ros_node.cancel_nav_goal()
        return "RUNNING"

# [신규] 모든 안전망을 통과했을 때만 실행되는 최후의 복구 노드
class ActionGlobalRecovery(BTNode):
    def tick(self, blackboard, ros_node):
        # 1. 여기까지 살아서 내려왔는데, 로봇 상태가 CANCELING(정지)에 묶여있다면?
        # 2. 그리고 가야 할 목적지(goal_name)가 여전히 남아있다면?
        if blackboard.goal_state == GoalState.CANCELING and blackboard.goal_name != "":
            ros_node.set_goal_state(GoalState.IDLE)
            
        # 상태만 풀어주고, 진짜 주행(Nav2 전송)은 다음 순위가 하도록 무조건 FAILURE를 뱉고 비켜줍니다.
        return "FAILURE"

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node):
        # 🛡️ [핵심 가드] 서비스 중(목적지가 있음)이 아니면 대상이 멀어지든 말든 신경 쓰지 않습니다.
        if blackboard.goal_name == "":
            return "FAILURE"
        
        # ros_node.get_logger().info(f"현재 상태입니다{blackboard.human_far}")
        return "SUCCESS" if getattr(blackboard, "human_far", False) else "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn("📢 [대기] 가이드 대상 거리 이탈. 추격 대기 모드 진입.", throttle_duration_sec=3.0)
        ros_node.cancel_nav_goal()
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
        # 🛠️ [핵심 수정] Nav2 목표를 보내기 직전에 상태를 RUNNING으로 변경
        # 이 변경으로 인해 다음 Tick부터 ConditionHasGoal 조건이 FAILURE가 되어 이 노드가 중복 호출되지 않습니다.
        blackboard.goal_state = GoalState.RUNNING
        # Nav2 액션 서버로 목표 전송 (내부적으로 SENT 상태 기록 등 수행)
        ros_node.send_nav_goal(blackboard.goal_x, blackboard.goal_y)
        return "RUNNING"
    

# qr !!
class ConditionQrAvailable(BTNode):
    def tick(self, blackboard, ros_node):
        # 기존 주행 목표가 완벽히 비어있고, 수신된 QR 데이터가 대기 중일 때만 동작
        has_qr_data = hasattr(blackboard, "qr_route_backup") and blackboard.qr_route_backup
        if blackboard.goal_name == "" and has_qr_data:
            return "SUCCESS"
        
        if blackboard.goal_name != "" and hasattr(blackboard, "qr_route_backup") and blackboard.qr_route_backup:
            # [로그 보완] 어떤 목적지 데이터가 씹혔는지 명시적으로 출력
            rejected_target = blackboard.qr_route_backup[0].get("location_name", "알 수 없음")
            ros_node.get_logger().warn(
                f"[QR 씹기] 기존 주행 스케줄('{blackboard.goal_name}')이 존재하므로 "
                f"수신된 QR 요청('{rejected_target}')을 폐기합니다.", 
                throttle_duration_sec=1.0
            )
            blackboard.qr_route_backup = None # 데이터 폐기
            
        return "FAILURE"
    

class ActionExecuteQrCall(BTNode):
    def tick(self, blackboard, ros_node):
        # 대기 중이던 QR 데이터를 정식 주행 경로로 승격
        qr_route = blackboard.qr_route_backup
        
        blackboard.web_route_list = qr_route  # 주행 리스트에 씌움 
        blackboard.current_waypoint_index = 0
        blackboard.navigation_finished = False
        
        # 웹 화면에 현재 상태가 QR 주행 중임을 리포트하기 위해 액션명 동기화
        blackboard.web_action = "qr_call_navigation"
        
        # Flask에서 정의한 딕셔너리 스키마 구조 추출 ("location_name", "x", "y")
        first_wp = qr_route[0]
        blackboard.goal_name = first_wp.get("location_name", "QR 목적지")
        blackboard.goal_x = float(first_wp.get("x", 0.0))
        blackboard.goal_y = float(first_wp.get("y", 0.0))
        
        # 상태를 IDLE로 변환하여 다음 틱에서 nav_br(ConditionHasGoal)이 인식하고 움직이도록 유도
        ros_node.set_goal_state(GoalState.IDLE)
        
        # 처리가 완료된 백업 변수 리셋
        blackboard.qr_route_backup = None
        return "SUCCESS"

# 7. 기본 정적 대기 브랜치
class ActionIdle(BTNode):
    def tick(self, blackboard, ros_node):
        return "RUNNING"