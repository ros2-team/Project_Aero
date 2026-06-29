#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose  
from rclpy.action import ActionClient  
import time
import threading  # 🎯 터미널 입력을 백그라운드에서 감시하기 위해 추가

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
        if blackboard.battery_low or blackboard.battery_level <= 30.0:  
            return "SUCCESS"
        return "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        if not blackboard.charging_started:   
            ros_node.get_logger().error(f"배터리 부족 상태 감지 ({blackboard.battery_level}%) -> 충전소 이동")
            ros_node.send_nav_goal(-0.07, -0.5)
            blackboard.charging_started = True
            blackboard.goal_sent = True 
        return "RUNNING"


####### 1. 하드웨어 무결성 감시 (센서 유실 감시) #######
class ConditionSensorTimeout(BTNode):
    def tick(self, blackboard, ros_node):
        current_time = time.time()
        if (current_time - blackboard.last_sensor_time) > 1.0 or blackboard.sensor_timeout:
            blackboard.sensor_timeout = True 
            return "SUCCESS"
        return "FAILURE"

class ActionSensorEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("⚠️ [CRITICAL] 센서 데이터 유실 감지! 시스템 정지 대기.", throttle_duration_sec=2.0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"
    
####### 1.5. 웹 인터페이스 강제 일시정지 브랜치 #######
class ConditionWebPause(BTNode):
    def tick(self, blackboard, ros_node):
        if getattr(blackboard, 'is_paused', False):
            return "SUCCESS"
        return "FAILURE"

class ActionWebPauseStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn("⏸️ [WEB] 사용자가 웹에서 일시정지를 요청했습니다. 강제 정지 유지.", throttle_duration_sec=3.0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"
    

####### 2. 전방 충돌 방지 -> 동적 장애물(움직이는 사람)일 때 즉각 비상정지 #######
class ConditionEmergency(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.is_dynamic_obstacle:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("[전방 위험] 동적 장애물 감지! 강제 정지.", throttle_duration_sec=1.0)
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"


####### 3. 후방캠 사람 추적 및 낙오 방지 #######
class ConditionHumanLost(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.human_lost or (not blackboard.human_tracked and blackboard.human_lost_timer >= 5.0):
            return "SUCCESS"
        return "FAILURE"

class ActionSearchHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().error("❓ [안내 유실] 대상 재탐색 대기 모드.", throttle_duration_sec=3.0)
        ros_node.publish_velocity(0.0, 0.0) 
        return "RUNNING"

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node): 
        if blackboard.human_far or (blackboard.human_tracked and blackboard.human_distance > 2.0):
            return "SUCCESS"
        return "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn(f"📢 [대기] 가이드 대상 거리 초과 ({blackboard.human_distance}m). 사용자를 기다립니다.", throttle_duration_sec=3.0)
        ros_node.publish_velocity(0.0, 0.0) 
        return "RUNNING"


####### 4. 측후방 회피 시간 보장 브랜치 (장애물 우회 제어 레이어) #######
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.obstacle_warning or (0.0 < blackboard.rear_obstacle_distance <= 1.5):
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().info("🔄 [우회 제어] 측후방 사물(차량 등) 발견 영역 통과 중. Nav2 Local Planner 가동을 보장합니다.", throttle_duration_sec=2.0)
        return "RUNNING"

####### 5. 경유지 도착 및 대기 제어 브랜치 #######
class ConditionArrived(BTNode):   
    def tick(self, blackboard, ros_node):
        if blackboard.is_arrived:
            return "SUCCESS"
        return "FAILURE"
    
class ActionStopGuide(BTNode):    
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.0, 0.0)
        return "RUNNING"     

####### 6. 내비게이션 최종 주행 실행 브랜치 (자율주행 제어 핵심 레이어) #######
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.has_goal and not blackboard.goal_failed:
            return "SUCCESS"
        return "FAILURE" 

class ActionMoveToGoal(BTNode):     
    def tick(self, blackboard, ros_node):
        if blackboard.goal_sent:
            return "RUNNING"

        ros_node.get_logger().info(f"🚀 이동 시작 -> {blackboard.goal_name}")
        ros_node.send_nav_goal(blackboard.goal_x, blackboard.goal_y)  
        blackboard.goal_sent = True
        return "RUNNING"


####### [BRANCH 7] 시스템 기저 베이스라인 대기 브랜치 (Default Idle Layer) #######
class ActionIdle(BTNode):
    def tick(self, blackboard, ros_node):
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

        self.root = Selector("Root")

        battery_branch = Sequence("BatteryBranch")
        battery_branch.add_child(ConditionBatteryLow("BatteryLow"))
        battery_branch.add_child(ActionSystemShutdown("Shutdown"))

        sensor_branch = Sequence("SensorBranch")
        sensor_branch.add_child(ConditionSensorTimeout("SensorTimeout"))
        sensor_branch.add_child(ActionSensorEmergencyStop("SensorEStop"))

        pause_branch = Sequence("WebPauseBranch")
        pause_branch.add_child(ConditionWebPause("WebPause"))
        pause_branch.add_child(ActionWebPauseStop("WebPauseStop"))

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

        self.root.add_child(battery_branch)        
        self.root.add_child(sensor_branch)     
        self.root.add_child(pause_branch) 
        self.root.add_child(emergency_branch)   
        self.root.add_child(human_control_hub)        
        self.root.add_child(avoid_branch)       
        self.root.add_child(arrival_branch)     
        self.root.add_child(nav_branch)        
        self.root.add_child(ActionIdle("SystemIdle"))

        self.timer = self.create_timer(0.1, self.bt_tick)

        self.input_thread = threading.Thread(target=self.terminal_input_loop, daemon=True)
        self.input_thread.start()

    # 🎯 [수정 완료 구역] 터미널 입력 감시 루프 함수
    def terminal_input_loop(self):
        while rclpy.ok():
            user_input = input("\n[TEST INJECTION] 일시정지 하려면 'y', 해제하려면 'n'을 입력하세요: ").strip().lower()
            
            if user_input == 'y':
                self.blackboard.is_paused = True
                self.get_logger().info("📥 [키보드 입력] blackboard.is_paused = True 주입 완료")
            elif user_input == 'n':
                # 🎯 팩트: 일시정지 스위치를 내리면서, 하단 주행 락 가드 플래그(goal_sent)도 같이 False로 해제합니다.
                self.blackboard.is_paused = False
                self.blackboard.goal_sent = False  
                self.get_logger().info("📥 [키보드 입력] blackboard.is_paused = False 및 goal_sent = False 주입 완료 (재출발)")
            else:
                print("⚠️ 잘못된 입력입니다. 'y' 또는 'n'만 입력하세요.")
                

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