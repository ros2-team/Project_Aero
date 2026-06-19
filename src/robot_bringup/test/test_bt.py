#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose  
from rclpy.action import ActionClient  

from battery_node import BatteryNode
from blackboard import Blackboard

# BT 기본 클래스
class BTNode:
    def __init__(self, name):
        self.name = name

    def tick(self, blackboard, ros_node):
        raise NotImplementedError

# Selector
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

# Sequence
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

# 1. Battery Branch
class ConditionBatteryLow(BTNode):
    def tick(self, blackboard, ros_node):
        # 실전 구동 시에는 30 이하로 변경해야 다른 브랜치들이 작동합니다.
        # 현재는 테스트를 위해 100 이하로 설정된 상태 확인 로그 제거/조정 가능
        if blackboard.battery_level <= 100.0:  
            return "SUCCESS"
        return "FAILURE"

class ActionSystemShutdown(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn(f"배터리 브랜치 구동 중! 충전 플래그 상태 = {blackboard.charging_started}", throttle_duration_sec=3.0)

        if not blackboard.charging_started:   
            ros_node.get_logger().warn(f"🔋 배터리 부족 상태 감지 ({blackboard.battery_level}%) -> 충전소 goal 전송")
            ros_node.send_nav_goal(-0.07, -0.5)
            blackboard.charging_started = True

        return "RUNNING"

# 2. Emergency Branch
class ConditionEmergency(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.obstacle_distance <= 0.5:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.0, 0.0)
        ros_node.get_logger().error("긴급 정지! 전방에 장애물 감지", throttle_duration_sec=1.0)
        return "RUNNING"

# 3. Avoidance Branch
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        dist = blackboard.obstacle_distance
        if 0.5 < dist <= 1.5:
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        direction = blackboard.obstacle_direction
        if direction == "LEFT":
            ros_node.publish_velocity(0.05, -0.3)  
        elif direction == "RIGHT":
            ros_node.publish_velocity(0.05, 0.3)   
        else:
            ros_node.publish_velocity(0.03, 0.0)   
        ros_node.get_logger().info("장애물 주행 회피 중", throttle_duration_sec=2.0)
        return "RUNNING"

# 4. Human Follow Check Branch
class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.human_tracked and blackboard.human_distance > 2.0:
            return "SUCCESS"
        return "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn(
            f"[안내 대기] 사람이 멀어졌습니다 (거리: {blackboard.human_distance}m). '이쪽으로 오세요' 신호 송신 중...",
            throttle_duration_sec=3.0
        )
        ros_node.publish_velocity(0.0, 0.0)  
        return "RUNNING"

# 5. Arrival Branch
class ConditionArrived(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.is_arrived:
            return "SUCCESS"
        return "FAILURE"

class ActionStopGuide(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.0, 0.0)
        ros_node.get_logger().info(f"목적지 [{blackboard.goal_name}] 도착 완료. 정지 후 다음 명령을 대기합니다.", throttle_duration_sec=5.0)
        return "RUNNING"

# 6. Navigation Branch
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard.has_goal:
            return "SUCCESS"
        return "FAILURE"

class ActionMoveToGoal(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.15, 0.0)  
        ros_node.get_logger().info(f"공항 안내 중 → 목적지: {blackboard.goal_name}", throttle_duration_sec=2.0)
        return "RUNNING"


# ROS2 행동트리 구동 노드 주체
class AirportGuideBT(Node):
    def __init__(self, blackboard):
        super().__init__("airport_guide_bt")

        self.blackboard = blackboard
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # 트리 구조 빌드 
        self.root = Selector("Root")

        # 1. 배터리 부족 브랜치
        battery_branch = Sequence("Battery")
        battery_branch.add_child(ConditionBatteryLow("BatteryLow"))
        battery_branch.add_child(ActionSystemShutdown("Shutdown"))

        # 2. 긴급 정지 브랜치
        emergency_branch = Sequence("Emergency")
        emergency_branch.add_child(ConditionEmergency("Emergency"))
        emergency_branch.add_child(ActionEmergencyStop("Stop"))

        # 3. 장애물 회피 브랜치
        avoid_branch = Sequence("Avoidance")
        avoid_branch.add_child(ConditionObstacle("Obstacle"))
        avoid_branch.add_child(ActionAvoidance("Avoid"))

        # 4. 사람 대기/신호 브랜치
        human_branch = Sequence("HumanFollow")
        human_branch.add_child(ConditionHumanFar("HumanFar"))
        human_branch.add_child(ActionSignalToHuman("Signal"))

        # 5. 목적지 도착 브랜치
        arrival_branch = Sequence("Arrival")
        arrival_branch.add_child(ConditionArrived("Arrived"))
        arrival_branch.add_child(ActionStopGuide("StopGuide"))

        # 6. 평시 목적지 이동 브랜치
        nav_branch = Sequence("Navigation")
        nav_branch.add_child(ConditionHasGoal("HasGoal"))
        nav_branch.add_child(ActionMoveToGoal("Move"))

        # 우선순위 등록
        self.root.add_child(battery_branch)    
        self.root.add_child(emergency_branch)  
        self.root.add_child(avoid_branch)      
        self.root.add_child(human_branch)      
        self.root.add_child(arrival_branch)    
        self.root.add_child(nav_branch)        

        # 0.1초 타이머 주기 실행
        self.timer = self.create_timer(0.1, self.bt_tick)

    def send_nav_goal(self, x, y):
        self.get_logger().error(f"★★★★ GOAL 함수 호출 ★<<< ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Nav2 서버 연결 대기 중...")
        self.nav_client.wait_for_server()
        self.get_logger().info(f"충전소 이동 액션 목표 전송 완료 ({x}, {y})")
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
    
    # 1. 하나의 공유 자원인 블랙보드 인스턴스 생성
    shared_blackboard = Blackboard()
    
    # 2. 각 노드에 공유 블랙보드 주입하며 생성
    bt_node = AirportGuideBT(shared_blackboard)
    battery_node = BatteryNode(shared_blackboard)
    
    # 3. 멀티스레드 이그제큐터를 생성하여 두 노드가 데이터를 공유하며 동시 병렬 스핀하도록 구동
    executor = MultiThreadedExecutor()
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        battery_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()