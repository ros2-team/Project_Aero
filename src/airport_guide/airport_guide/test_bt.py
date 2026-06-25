#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose  
from rclpy.action import ActionClient  
import time

from battery_node import BatteryNode
from blackboard import Blackboard
from arrival_node import ArrivalNode

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
        # 자식 노드 중 하나라도 SUCCESS나 RUNNING을 반환하면 즉시 실행을 멈추고 그 상태를 부모에게 보고합니다.
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
        # 자식 노드가 차례대로 SUCCESS를 반환해야만 다음 자식으로 넘어갑니다.
        for child in self.children:  
            status = child.tick(blackboard, ros_node)
            if status != "SUCCESS":  
                return status  
        return "SUCCESS"  

# ==============================================================================
# [BRANCH 0] Battery Low
# ==============================================================================
class ConditionBatteryLow(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.battery_level <= 30.0:  
            return "SUCCESS"
        return "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        if not blackboard.charging_started:   
            ros_node.get_logger().error(f"🚨 배터리 부족 상태 감지 ({blackboard.battery_level}%) -> 충전소 이동")
            ros_node.send_nav_goal(-0.07, -0.5)
            blackboard.charging_started = True
        return "RUNNING"

# ==============================================================================
# [BRANCH 1] Sensor Timeout
# ==============================================================================
class ConditionSensorTimeout(BTNode):
    def tick(self, blackboard, ros_node):
        current_time = time.time()
        if (current_time - blackboard.last_sensor_time) > 1.0 or blackboard.sensor_timeout:
            return "SUCCESS"
        return "FAILURE"

class ActionSensorEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "STOPPED_BY_SENSOR_TIMEOUT"
        ros_node.get_logger().error("⚠️ [CRITICAL] 센서 데이터 유실 감지! 시스템 정지 대기.", throttle_duration_sec=2.0)
        return "RUNNING"

# ==============================================================================
# [BRANCH 2] Emergency Stop
# ==============================================================================
class ConditionEmergency(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.obstacle_distance <= 0.5:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "EMERGENCY_STOP"
        ros_node.get_logger().error("🛑 긴급 정지 상태 활성화!", throttle_duration_sec=1.0)
        return "RUNNING"

# ==============================================================================
# [BRANCH 3] Human Tracker Control
# ==============================================================================
class ConditionHumanLost(BTNode):
    def tick(self, blackboard, ros_node):
        if not blackboard.human_tracked and blackboard.human_lost_timer >= 5.0:
            return "SUCCESS"
        return "FAILURE"

class ActionSearchHuman(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "HUMAN_LOST"
        ros_node.get_logger().error("❓ [안내 유실] 대상 재탐색 대기 모드.", throttle_duration_sec=3.0)
        return "RUNNING"

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node): 
        if blackboard.human_tracked and blackboard.human_distance > 2.0:
            return "SUCCESS"
        return "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "WAITING_HUMAN"
        ros_node.get_logger().warn(f"📢 [대기] 가이드 대상 거리 초과 ({blackboard.human_distance}m).", throttle_duration_sec=3.0)
        return "RUNNING"

# ==============================================================================
# [BRANCH 4] Avoidance
# ==============================================================================
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        if 0.5 < blackboard.obstacle_distance <= 1.5:
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "LOCAL_AVOIDANCE"
        ros_node.get_logger().info("🔄 회피 제어 모드 활성화", throttle_duration_sec=2.0)
        return "RUNNING"

# ==============================================================================
# [BRANCH 5] Arrival (도착 및 정지 제어 브랜치)
# ==============================================================================
class ConditionArrived(BTNode):   
    def tick(self, blackboard, ros_node):
        # arrival_node가 목적지 0.2m 안으로 들어와서 플래그를 True로 켰는지 검사합니다.
        if blackboard.is_arrived:
            return "SUCCESS" # 도착했으므로 우측의 ActionStopGuide를 실행시킵니다.
        return "FAILURE"     # 아직 주행 중이거나 대기가 끝나 리셋된 상태라면 실패를 뱉어 이 브랜치를 닫습니다.
    
class ActionStopGuide(BTNode):    
    def tick(self, blackboard, ros_node):
        # arrival_node가 5초를 다 세고 다음 목적지를 갱신하면서 플래그를 꺼줄 때까지 
        # 매 틱(0.1초)마다 반복해서 바퀴에 정지 명령(0,0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"     
        # 이 브랜치가 열려있는 동안(대기 5초 동안)은 트리의 아래 흐름(6번)으로 내려가지 못하게 묶어둡니다.
    

# ==============================================================================
# [BRANCH 6] Navigation (새 목적지)
# ==============================================================================
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        # 아직 리스트에 남은 경유지가 있거나 주행할 미션이 살아있는지 검사합니다.
        if blackboard.has_goal:
            return "SUCCESS"
        return "FAILURE" 

class ActionMoveToGoal(BTNode):     
    def __init__(self, name):
        super().__init__(name)
        self.start_track_time = None

    def tick(self, blackboard, ros_node):
        # 1. 상태가 IDLE이면 -> 새 좌표를 Nav2에 딱 한 번 발사
        if blackboard.nav_status == "IDLE":
            ros_node.get_logger().info(f"🚀 이동 시작 -> {blackboard.goal_name}")
            ros_node.send_nav_goal(blackboard.goal_x, blackboard.goal_y)  
            
            blackboard.nav_status = "NAV_STARTING"
            self.start_track_time = time.time()
            return "RUNNING"

        # 2. Nav2 액션 서버 접수 및 오도메트리 안착 지연 시간 벌어주기 (0.5초)
        if blackboard.nav_status == "NAV_STARTING":
            if (time.time() - self.start_track_time) < 0.5:
                return "RUNNING"
            
            ros_node.get_logger().info("🟢 물리 주행 상태 추적 시작.")
            blackboard.nav_status = "EXECUTING"
            return "RUNNING" # 💡 기존 SUCCESS에서 RUNNING으로 변경!

        # 3. 💡 핵심: 로봇이 주행하는 내내 RUNNING을 뱉어줌으로써, 
        # 트리가 맨 밑바닥 ActionIdle로 추락해서 nav_status를 IDLE로 깨부수는 것을 원천 차단합니다.
        if blackboard.nav_status == "EXECUTING":
            return "RUNNING"

        return "FAILURE"

# ==============================================================================
# [BRANCH 7] Base Baseline Idle
# ==============================================================================
class ActionIdle(BTNode):
    def tick(self, blackboard, ros_node):
        blackboard.nav_status = "IDLE"
        ros_node.get_logger().info("💤 트리 베이스라인 대기 모드", throttle_duration_sec=10.0)
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

        self.root = Selector("Root")

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
        nav_branch.add_child(ConditionHasGoal("HasGoal"))     # 1. 목표가 남았는가?
        nav_branch.add_child(ActionMoveToGoal("MoveToGoal"))   # 2. 🎯 IDLE 상태면 새 목적지로 발사!

        # 우선순위(위->아래)에 맞게 차례대로 등록
        self.root.add_child(battery_branch)    # 0번 브랜치    
        self.root.add_child(sensor_branch)     # 1번 브랜치 
        self.root.add_child(emergency_branch)  # 2번 브랜치 
        self.root.add_child(human_control_hub) # 3번 브랜치       
        self.root.add_child(avoid_branch)      # 4번 브랜치 
        self.root.add_child(arrival_branch)    # 5번 브랜치 
        self.root.add_child(nav_branch)        # 6번 브랜치
        self.root.add_child(ActionIdle("SystemIdle"))

        self.timer = self.create_timer(0.1, self.bt_tick)

    def send_nav_goal(self, x, y):   # Nav2한테 목적지 보내는 코드
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"  # map 좌표기준으로 () 가라  
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()  # nav 액션서버 대기 
        self.nav_client.send_goal_async(goal_msg)  # 목표 전송 

    
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
    
    shared_blackboard.wait_started = False
    shared_blackboard.wait_start_time = 0.0
    
    bt_node = AirportGuideBT(shared_blackboard)
    battery_node = BatteryNode(shared_blackboard)
    arrival_node = ArrivalNode(shared_blackboard) 
    
    executor = MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    executor.add_node(arrival_node) 
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        battery_node.destroy_node()
        arrival_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()