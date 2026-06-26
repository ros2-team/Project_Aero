#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose  
from rclpy.action import ActionClient  
import time

# 라이브러리 및 노드 의존성 임포트
from airport_guide.battery_node import BatteryNode
from airport_guide.blackboard import Blackboard
from airport_guide.arrival_node import ArrivalNode
from airport_guide.front_cam_node import FrontCameraNode

# ==============================================================================
# [CORE] 행동 트리 기저 클래스 정의
# ==============================================================================
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

######## 0. 배터리 방전 방지 브랜치 #######
class ConditionBatteryLow(BTNode):
    def tick(self, blackboard, ros_node):
        # 배터리 관리 노드가 동기화하는 상태 플래그 및 수치를 독립 검사합니다.
        if blackboard.battery_low or blackboard.battery_level <= 30.0:  
            return "SUCCESS"
        return "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        # 중복 발사 잠금: 복귀 명령이 한 번 전달되었다면 중복 액션을 차단합니다.
        if not blackboard.charging_started:   
            ros_node.get_logger().error(f"배터리 부족 상태 감지 ({blackboard.battery_level}%) -> 충전소 이동")
            ros_node.send_nav_goal(-0.07, -0.5)
            blackboard.charging_started = True
            blackboard.goal_sent = True # 충전소 주행 목표 또한 일반 내비게이션 목표 발사 상태로 묶어 잠금
        return "RUNNING"


####### 1. 하드웨어 무결성 감시 (센서 유실 감시) #######
# 모터 오도메트리나 메인 통신 패킷이 끊겼을 때 로봇이 엇나가지 않도록 강제 제동을 거는 구역
class ConditionSensorTimeout(BTNode):
    def tick(self, blackboard, ros_node):
        current_time = time.time()
        # 최종 수신 타임스탬프 간격이 1초를 초과하거나 명시적 에러 신호가 감지되면 참
        if (current_time - blackboard.last_sensor_time) > 1.0 or blackboard.sensor_timeout:
            blackboard.sensor_timeout = True 
            return "SUCCESS"
        return "FAILURE"

class ActionSensorEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        # 센서 레이어가 복구될 때까지 진행 방향 속도를 완전히 소거하여 대기 상태를 유지
        ros_node.get_logger().error("⚠️ [CRITICAL] 센서 데이터 유실 감지! 시스템 정지 대기.", throttle_duration_sec=2.0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"


####### 2. 전방 충돌 방지 -> 전방 카메라 인지 및 비상정지 #######
class ConditionEmergency(BTNode):
    def tick(self, blackboard, ros_node):
        # 전방 카메라(YOLO) 인간 플래그 및 라이다 장애물 검출 플래그가 작동되면 넘김 
        if blackboard.is_front_human or blackboard.obstacle_detected:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        # 비상 정지 상태 시 전방 장애가 완전 해제될 때까지 모터 출력을 0으로 강력 주입
        ros_node.get_logger().error("[전방 위험] 비상 정지 조건 충족! 강제 정지.", throttle_duration_sec=1.0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"

####### 3. 후방캠 사람 추적 및 낙오 방지 #######
class ConditionHumanLost(BTNode):
    def tick(self, blackboard, ros_node):
        # 사용자 완전히 식별 불가능하거나, 트래킹 유실 한계 시간(5.0초)을 넘어섰을 때 true
        if blackboard.human_lost or (not blackboard.human_tracked and blackboard.human_lost_timer >= 5.0):
            return "SUCCESS"
        return "FAILURE"

class ActionSearchHuman(BTNode):
    def tick(self, blackboard, ros_node):
        # 사용자 놓차면 무작정 전진하지 않고 주행을 즉시 멈춘 뒤 제자리에서 탐색 대기
        ros_node.get_logger().error("❓ [안내 유실] 대상 재탐색 대기 모드.", throttle_duration_sec=3.0)
        ros_node.publish_velocity(0.0, 0.0) 
        return "RUNNING"

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node): 
        # 트래킹은 유지 중이나 사용자가 걸음이 느려 로봇과 2.0m 이상 벌어지면 발동
        if blackboard.human_far or (blackboard.human_tracked and blackboard.human_distance > 2.0):
            return "SUCCESS"
        return "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        # 낙오 방지: 사용자가 뒤에서 안전거리 안으로 다시 좁혀 다가올 때까지 전진 주행을 일시정지하고 기다려줍니다.
        ros_node.get_logger().warn(f"📢 [대기] 가이드 대상 거리 초과 ({blackboard.human_distance}m). 사용자를 기다립니다.", throttle_duration_sec=3.0)
        ros_node.publish_velocity(0.0, 0.0) 
        return "RUNNING"


####### 4. 측후방 회피 시간 보장 브랜치 (장애물 우회 제어 레이어) #######
# 라이다 사각지대나 측후방에 사물 발견 시, Nav2 내부 경로 생성기(Local Planner)가 방해받지 않고 스스로 우회하도록 시간을 주는 구역입니다.
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        # 라이다 기반 우회 경고 플래그 상태나 후방 위험 물체 거리(1.5m 이내)를 검사합니다.
        if blackboard.obstacle_warning or (0.0 < blackboard.rear_obstacle_distance <= 1.5):
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        # 여기서는 속도 명령(publish_velocity)을 직접 내리지 않음.
        # 트리는 오직 상위 우선순위 영역에서 'RUNNING'만 리턴하여 하위의 목표 재전송 루프를 차단하고, Nav2가 유연하게 우회할 수 있도록 함.
        ros_node.get_logger().info("🔄 [우회 제어] 사물(차량 등) 발견 영역 통과 중. Nav2 Local Planner 가동을 보장합니다.", throttle_duration_sec=2.0)
        return "RUNNING"


####### 5. 경유지 도착 및 대기 제어 브랜치 #######
# 독립 노드인 arrival_node와 연결 
class ConditionArrived(BTNode):   
    def tick(self, blackboard, ros_node):
        # 외부 정밀 도달 연산 노드가 켜주는 'is_arrived' 스위치를 단순 판독합니다.
        if blackboard.is_arrived:
            return "SUCCESS"
        return "FAILURE"
    
class ActionStopGuide(BTNode):    
    def tick(self, blackboard, ros_node):
        # arrival_node가 지정한 시간(5초) 동안 타이머 연산을 모두 마치고 플래그를 꺼줄 때까지,
        # 트리가 아래 주행 레이어(6번)로 새 목표를 쏘지 못하게 RUNNING으로 묶어두며 바퀴를 제동해 세웁니다.
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"     

####### 6. 내비게이션 최종 주행 실행 브랜치 (자율주행 제어 핵심 레이어) #######
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        # 미션 경유지가 블랙보드에 장전되어 있고 시스템 실패 상태가 아니어야 진입을 허용합니다.
        if blackboard.has_goal and not blackboard.goal_failed:
            return "SUCCESS"
        return "FAILURE" 

class ActionMoveToGoal(BTNode):     
    def tick(self, blackboard, ros_node):
        # [중복 전송 버그를 원천 봉쇄하는 락 가드 플래그 연산]
        # 이미 최초 1회 목적지 좌표를 Nav2 액션 서버로 전송 완료한 상태라면 (`goal_sent == True`),
        if blackboard.goal_sent:
            return "RUNNING"

        # goal_sent가 False라는 뜻은 최초 주행을 시작하는 순간이거나 대기 타이머가 끝나 새 경유지를 인계받은 타이밍
        ros_node.get_logger().info(f"🚀 이동 시작 -> {blackboard.goal_name}")
        
        # ROS2 Action 클라이언트를 통해 Nav2 코어 서버로 목표 Pose 패킷 비동기 송신 (오직 최초 1회만 트러거됨)
        ros_node.send_nav_goal(blackboard.goal_x, blackboard.goal_y)  
        
        # 송신 직후 잠금장치 스위치를 True로 올려 다음 0.1초 틱 주기 때 중복 호출되는 심각한 네트워크 부하 버그를 차단
        blackboard.goal_sent = True
        return "RUNNING"


####### [BRANCH 7] 시스템 기저 베이스라인 대기 브랜치 (Default Idle Layer) #######
# 모든 미션 스케줄을 정상 수행하여 갈 곳이 없거나 유휴 공백 상태일 때 진입하는 안전지대입니다.
class ActionIdle(BTNode):
    def tick(self, blackboard, ros_node):
        # 예외 상황 및 트리 크래시를 온전히 마스킹하고 대기 상태 가동 정보를 정기 출력합니다.
        ros_node.get_logger().info("💤 트리 베이스라인 대기 모드 (모든 태스크 완료 혹은 대기)", throttle_duration_sec=10.0)
        return "RUNNING"


# ==============================================================================
# [NODE SYSTEM] ROS2 메인 엔진 오케스트레이터
# ==============================================================================
class AirportGuideBT(Node):
    def __init__(self, blackboard):
        super().__init__("airport_guide_bt")
        self.blackboard = blackboard
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 최상위 제어 분기점 루트 노드 초기화
        self.root = Selector("Root")

        # 각 시나리오별 조건/액션 결합 파이프라인 조립
        battery_branch = Sequence("BatteryBranch")
        battery_branch.add_child(ConditionBatteryLow("BatteryLow"))
        battery_branch.add_child(ActionSystemShutdown("Shutdown"))

        sensor_branch = Sequence("SensorBranch")
        sensor_branch.add_child(ConditionSensorTimeout("SensorTimeout"))
        sensor_branch.add_child(ActionSensorEmergencyStop("SensorEStop"))

        emergency_branch = Sequence("EmergencyBranch")
        emergency_branch.add_child(ConditionEmergency("Emergency"))
        emergency_branch.add_child(ActionEmergencyStop("Stop"))

        human_control_hub = Selector("HumanControlHub")
        human_lost_seq = Sequence("HumanLostSeq")
        human_lost_seq.add_child(ConditionHumanLost("HumanLost"))
        human_lost_seq.add_child(ActionSearchHuman("SearchHuman"))
        human_far_seq = Sequence("HumanFarSeq")
        human_far_seq.add_child(ConditionHumanFar("HumanFar"))
        human_far_seq.add_child(ActionSignalToHuman("SignalToHuman"))
        human_control_hub.add_child(human_lost_seq)
        human_control_hub.add_child(human_far_seq)

        avoid_branch = Sequence("AvoidanceBranch")
        avoid_branch.add_child(ConditionObstacle("Obstacle"))
        avoid_branch.add_child(ActionAvoidance("Avoid"))
        
        arrival_branch = Sequence("ArrivalBranch")
        arrival_branch.add_child(ConditionArrived("Arrived"))
        arrival_branch.add_child(ActionStopGuide("StopGuide"))
        
        nav_branch = Sequence("NavigationBranch")
        nav_branch.add_child(ConditionHasGoal("HasGoal"))     
        nav_branch.add_child(ActionMoveToGoal("MoveToGoal"))   

        # [우선순위 구조 기반 배치] Selector 노드의 우선순위 법칙
        self.root.add_child(battery_branch)        
        self.root.add_child(sensor_branch)      
        self.root.add_child(emergency_branch)   
        self.root.add_child(human_control_hub)        
        self.root.add_child(avoid_branch)       
        self.root.add_child(arrival_branch)     
        self.root.add_child(nav_branch)        
        self.root.add_child(ActionIdle("SystemIdle"))

        # 0.1초 간격으로 전체 행동 트리를 순회 
        self.timer = self.create_timer(0.1, self.bt_tick)

    def send_nav_goal(self, x, y):   
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"  
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()  
        self.nav_client.send_goal_async(goal_msg)  

    def bt_tick(self):
        self.root.tick(self.blackboard, self)
    
    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    shared_blackboard = Blackboard()
    shared_blackboard.last_sensor_time = time.time()
    
    bt_node = AirportGuideBT(shared_blackboard)
    battery_node = BatteryNode(shared_blackboard)
    arrival_node = ArrivalNode(shared_blackboard) 
    front_cam_node = FrontCameraNode(shared_blackboard) 
    
    # 멀티스레드 환경을 명시 선언하여 노드 간 비동기 Flag 쓰기/읽기 작업 시 데드락을 방지
    executor = MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    executor.add_node(arrival_node) 
    executor.add_node(front_cam_node) 
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        battery_node.destroy_node()
        arrival_node.destroy_node()
        front_cam_node.destroy_node() 
        rclpy.shutdown()

if __name__ == "__main__":
    main()