#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState # 배터리 토픽 받는 msg 타입
from nav2_msgs.action import NavigateToPose  # 액션 타입
from rclpy.action import ActionClient  # 액션 서버에 goal를 보내는 클라 


# BT 기본 클래스
class BTNode:
    def __init__(self, name):
        self.name = name

    def tick(self, blackboard, ros_node):
        raise NotImplementedError


# Selector (우선순위 노드: 자식 중 하나라도 SUCCESS/RUNNING이면 즉시 리턴)
class Selector(BTNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def tick(self, blackboard, ros_node):  
        for child in self.children:  # 자식들 순서대로 루프 
            status = child.tick(blackboard, ros_node)
            if status != "FAILURE":  #결과가 FAILURE가 아니면 
                return status  #종료
        return "FAILURE"  #자식 노드들이 다 FAILURE이면 Selector에서도 최종 FAILURE이면 리턴 

    # SUCCESS / RUNNING 이면 해당 브랜치가 로봇 제어권 독점 
    # FAILURE이면 다른 자식 시퀀스 브랜치로 넘어감 


# Sequence (순차 실행 노드: 자식이 전부 SUCCESS여야 다음으로 진행)
class Sequence(BTNode):
    def __init__(self, name):
        super().__init__(name)
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def tick(self, blackboard, ros_node):
        for child in self.children:  # 등록된 자식 노드 순서대로 실행
            status = child.tick(blackboard, ros_node)
            if status != "SUCCESS":  # 결과가 SUCCESS가 아니면 
                return status  # 뒤에 남은 노드 무시 
        return "SUCCESS"  # 자식이 반환해야 종료 



# 1. Battery Branch (최우선 순위: 30% 이하 시 시스템 미작동)
class ConditionBatteryLow(BTNode):
    def tick(self, blackboard, ros_node):

        ros_node.get_logger().info(
            f"현재 배터리 검사: {blackboard['battery_level']}"
        )

        if blackboard['battery_level'] <= 100:  # 임의로 100이하로 설정******
            return "SUCCESS"
        return "FAILURE"

class ActionSystemShutdown(BTNode):
    
    def tick(self, blackboard, ros_node):

        ros_node.get_logger().warn(
            f"charging_started = {blackboard['charging_started']}"
        ) 

        ros_node.get_logger().warn("배터리 브랜치 진입!")

        if not blackboard['charging_started']:   
            # 한 번만 goal 전송 why? tick 0.1초마다 도는데 goal 계쏙 보내면 정신나감

            ros_node.get_logger().warn(
                f"🔋 배터리 부족 ({blackboard['battery_level']}%)"
            )

            ros_node.send_nav_goal(   # 맵 기준 이 좌표로 가 
                -0.07, 
                -1.5
            )

            blackboard['charging_started'] = True

        return "RUNNING"


# 2. Emergency Branch (안전 확보: 0.5m 이내 장애물 발생 시 긴급 정지)
class ConditionEmergency(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard['obstacle_distance'] <= 0.5:
            return "SUCCESS"
        return "FAILURE"

class ActionEmergencyStop(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.0, 0.0)
        ros_node.get_logger().error("긴급 정지! 전방에 장애물 감지")
        return "RUNNING"



# 3. Avoidance Branch (충돌 회피: 0.5m 초과 1.5m 이내 장애물 제어)
class ConditionObstacle(BTNode):
    def tick(self, blackboard, ros_node):
        dist = blackboard['obstacle_distance']
        if 0.5 < dist <= 1.5:
            return "SUCCESS"
        return "FAILURE"

class ActionAvoidance(BTNode):
    def tick(self, blackboard, ros_node):
        direction = blackboard['obstacle_direction']
        if direction == "LEFT":
            ros_node.publish_velocity(0.05, -0.3)  # 우회전
        elif direction == "RIGHT":
            ros_node.publish_velocity(0.05, 0.3)   # 좌회전
        else:
            ros_node.publish_velocity(0.03, 0.0)   # 서행
        ros_node.get_logger().info("장애물 주행 회피 중")
        return "RUNNING"



# 4. Human Follow Check Branch (사람 추적: 2미터 밖에 있으면 대기 및 신호)

class ConditionHumanFar(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard['human_tracked'] and blackboard['human_distance'] > 2.0:
            return "SUCCESS"
        return "FAILURE"

class ActionSignalToHuman(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.get_logger().warn(
            f"[안내 대기] 사람이 멀어졌습니다 (거리: {blackboard['human_distance']}m). '이쪽으로 오세요' 신호 송신 중...",
            throttle_duration_sec=3.0
        )
        ros_node.publish_velocity(0.0, 0.0)  # 사람이 2m 이내로 들어올 때까지 정지하여 대기
        return "RUNNING"


# 5. Arrival Branch (도착 확인: 목적지에 도착했으면 정지)
class ConditionArrived(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard['is_arrived']:
            return "SUCCESS"
        return "FAILURE"

class ActionStopGuide(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.0, 0.0)
        ros_node.get_logger().info(f"목적지 [{blackboard['goal_name']}] 도착 완료. 정지 후 다음 명령을 대기합니다.")
        return "RUNNING"


# 6. Navigation Branch (평시 주행: 목적지가 있으면 이동)
class ConditionHasGoal(BTNode):
    def tick(self, blackboard, ros_node):
        if blackboard['has_goal']:
            return "SUCCESS"
        return "FAILURE"

class ActionMoveToGoal(BTNode):
    def tick(self, blackboard, ros_node):
        ros_node.publish_velocity(0.15, 0.0)  # 평시 직진 주행
        ros_node.get_logger().info(f"공항 안내 중 → 목적지: {blackboard['goal_name']}")
        return "RUNNING"


# ROS2 행동트리 구동 노드

class AirportGuideBT(Node):
    def __init__(self):
        super().__init__("airport_guide_bt")
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

     # 블랙보드 데이터 세팅 (센서/상태 데이터 저장소)

        self.blackboard = {
            'battery_level': 100,         # 배터리 잔량 
            'charging_started': False,    # 아직 충전소 goal 안보냄 
            
            'has_goal': True,             # 안내 목적지 존재 여부
            'goal_name': "Gate_A3",       # 목적지 명칭
            'is_arrived': False,          # 목적지 도착 여부
            
            'obstacle_distance': 10.0,    # 라이다 기준 장애물 거리 (m)
            'obstacle_direction': "CENTER",# 장애물 위치 (LEFT / RIGHT / CENTER)
            
            'human_tracked': True,        # 사람 인식 여부
            'human_distance': 1.2         # 사람과의 거리 (m)
        }


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


        # 우선순위 등록 순서
        self.root.add_child(battery_branch)    # 0순위: 배터리
        self.root.add_child(emergency_branch)  # 1순위: 긴급 정지
        self.root.add_child(avoid_branch)      # 2순위: 회피 주행
        self.root.add_child(human_branch)      # 3순위: 사람 거리 체크 및 오라고 신호
        self.root.add_child(arrival_branch)    # 4순위: 목적지 도착 확인
        self.root.add_child(nav_branch)        # 5순위: 평시 가이드 주행

        # 0.1초(10Hz) 주기 타이머 생성
        self.timer = self.create_timer(0.1, self.bt_tick)



    def send_nav_goal(self, x, y):

        self.get_logger().error(
        f"★★★★ GOAL 함수 호출 ★★★★ ({x}, {y})"
        )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()

        self.get_logger().info(
            f"충전소 이동 시작 ({x}, {y})"
        )

        self.nav_client.send_goal_async(goal_msg)

    
    def battery_callback(self, msg):
        self.blackboard['battery_level'] = msg.percentage   # 배터리 ㅍ센트 블랙보드에 저장  

        self.get_logger().info(
            f"Battery = {msg.percentage:.1f}%"
        )

    def bt_tick(self):
        # 최상단 루프 노드 실행
        self.root.tick(self.blackboard, self)

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AirportGuideBT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()