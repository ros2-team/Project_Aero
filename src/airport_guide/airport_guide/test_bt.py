#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# ---------------------------------------------------------------------
# 📦 아키텍처 핵심 노드 및 변환 노드 패키지 임포트
# ---------------------------------------------------------------------
from airport_guide.blackboard import Blackboard, GoalState
from airport_guide.web_data import WebBridgeNode                  # Flask 감시 노드

from airport_guide.bt_nodes import (
    Selector, Sequence,
    ConditionBatteryLow, ActionSystemShutdown,
    ConditionSensorTimeout, ActionSensorEmergencyStop,
    ConditionWebPause, ActionWebPauseStop,
    ConditionEmergency, ActionEmergencyStop,
    ActionRecoverFromEmergency,
    ConditionHumanLost, ActionSearchHuman,
    ConditionHumanFar, ActionSignalToHuman,
    ConditionObstacle, ActionAvoidance,
    ConditionArrived, ActionStopGuide,
    ConditionHasGoal, ActionMoveToGoal,
    ActionIdle,
)
from airport_guide.battery_node import BatteryNode
from airport_guide.arrival_node import ArrivalNode
from airport_guide.front_cam_node import FrontCameraNode
from airport_guide.web_pause import WebPauseNode


class AirportGuideBT(Node):
    def __init__(self, blackboard):
        super().__init__("airport_guide_bt")
        self.blackboard = blackboard
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self._current_goal_handle = None
        
        # ---------------------------------------------------------------------
        # 웹브릿지(web_data)로 실시간 상태를 쏴줄 ROS2 퍼블리셔 추가
        # ---------------------------------------------------------------------
        self.bt_status_pub = self.create_publisher(String, "/robot/bt_status", 10)

        self.pause_sub = self.create_subscription(
            Bool, '/test/pause', self._pause_callback, 10
        )

        self.root = Selector("Root")

        # 각 시나리오별 서브 브랜치 선언
        battery_br = Sequence("BatteryBranch")
        battery_br.add_child(ConditionBatteryLow("BatteryLow"))
        battery_br.add_child(ActionSystemShutdown("Shutdown"))

        sensor_br = Sequence("SensorBranch")
        sensor_br.add_child(ConditionSensorTimeout("SensorTimeout"))
        sensor_br.add_child(ActionSensorEmergencyStop("SensorEStop"))

        pause_br = Sequence("WebPauseBranch")
        pause_br.add_child(ConditionWebPause("WebPause"))
        pause_br.add_child(ActionWebPauseStop("WebPauseStop"))

        emergency_br = Selector("EmergencyBranch")
        stop_seq = Sequence("EmergencyStopSeq")
        stop_seq.add_child(ConditionEmergency("Emergency"))
        stop_seq.add_child(ActionEmergencyStop("Stop"))
        emergency_br.add_child(stop_seq)
        emergency_br.add_child(ActionRecoverFromEmergency("Recover"))

        human_hub = Selector("HumanControlHub")
        lost_seq = Sequence("HumanLostSeq")
        lost_seq.add_child(ConditionHumanLost("HumanLost"))
        lost_seq.add_child(ActionSearchHuman("SearchHuman"))
        far_seq = Sequence("HumanFarSeq")
        far_seq.add_child(ConditionHumanFar("HumanFar"))
        far_seq.add_child(ActionSignalToHuman("SignalToHuman"))
        human_hub.add_child(lost_seq)
        human_hub.add_child(far_seq)

        avoid_br = Sequence("AvoidanceBranch")
        avoid_br.add_child(ConditionObstacle("Obstacle"))
        avoid_br.add_child(ActionAvoidance("Avoid"))

        arrival_br = Sequence("ArrivalBranch")
        arrival_br.add_child(ConditionArrived("Arrived"))
        arrival_br.add_child(ActionStopGuide("StopGuide"))

        nav_br = Sequence("NavigationBranch")
        nav_br.add_child(ConditionHasGoal("HasGoal"))
        nav_br.add_child(ActionMoveToGoal("MoveToGoal"))

        # 최상위 Selector 자식 순서 배치
        self.root.add_child(nav_br) 
        self.root.add_child(battery_br)
        self.root.add_child(sensor_br)
        self.root.add_child(pause_br)
        self.root.add_child(emergency_br)
        self.root.add_child(human_hub)
        self.root.add_child(avoid_br)
        self.root.add_child(arrival_br)
        self.root.add_child(ActionIdle("SystemIdle"))

        self.timer = self.create_timer(0.1, self.bt_tick)

    def _pause_callback(self, msg):
        if msg.data:
            self.blackboard.is_paused = True
        else:
            self.blackboard.is_paused = False
            if self.blackboard.goal_state == GoalState.CANCELING:
                self.set_goal_state(GoalState.IDLE)

    def send_nav_goal(self, x, y):
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._current_goal_handle = None

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.set_goal_state(GoalState.SENT)

    def set_goal_state(self, new_state: GoalState):
        old_state = self.blackboard.goal_state
        self.blackboard.goal_state = new_state
        self.get_logger().info(f"🔁 [FSM] goal_state: {old_state.name} -> {new_state.name}")

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2가 goal을 거절했습니다.")
            self.set_goal_state(GoalState.IDLE)
            return

        self._current_goal_handle = goal_handle
        self.set_goal_state(GoalState.RUNNING)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        if self.blackboard.goal_state not in [GoalState.CANCELING, GoalState.IDLE]:
            self.set_goal_state(GoalState.DONE)
        self._current_goal_handle = None

    def cancel_nav_goal(self):
        if self.blackboard.goal_state in [GoalState.RUNNING, GoalState.SENT]:
            if self._current_goal_handle is not None:
                self.set_goal_state(GoalState.CANCELING)
                self._current_goal_handle.cancel_goal_async()
            else:
                self.set_goal_state(GoalState.IDLE)

    # *********** 디버깅 및 웹 상태 보고 코드 **********************
    def bt_tick(self):
        # 1초마다 블랙보드 상태를 터미널에 강제로 찍어주는 디버그 로그
        self.get_logger().info(
            f"🔄 [BT TICK] 현재 goal_name: '{self.blackboard.goal_name}', "
            f"state: {self.blackboard.goal_state.name if self.blackboard.goal_state else 'None'}", 
            throttle_duration_sec=1.0
        )

        # ---------------------------------------------------------------------
        # 📊 1초 주기로 전처리 노드들이 쌓아둔 종합 데이터를 취합하여 웹브릿지로 발행
        # ---------------------------------------------------------------------
        try:
            status_payload = {
                "goal_state": self.blackboard.goal_state.name if self.blackboard.goal_state else "UNKNOWN",
                "goal_name": self.blackboard.goal_name,
                "battery_low": getattr(self.blackboard, "battery_low", False),
                "battery_level": getattr(self.blackboard, "battery_level", 100.0),
                "odom_pose": {
                    # 🎯 blackboard.py 정의 변수명(current_x, current_y) 매핑 일치화
                    #  getattr(A, "B", C): "A 객체 안에서 'B'라는 이름의 실시간 변수 값을 가져오되, 만약 변수가 존재하지 않으면 기본값으로 C를 반환
                    "x": getattr(self.blackboard, "current_x", 0.0),
                    "y": getattr(self.blackboard, "current_y", 0.0),
                    "yaw": getattr(self.blackboard, "current_yaw", 0.0) 
                }
            }
            
            # JSON 직렬화 후 문자열 메시지로 변환 및 퍼블리시
            msg = String()
            msg.data = json.dumps(status_payload)
            self.bt_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"웹 피드백 데이터 취합 및 퍼블리시 실패: {e}", throttle_duration_sec=3.0)
        # ---------------------------------------------------------------------

        # 실제 행동트리 실행
        self.root.tick(self.blackboard, self)


def main(args=None):
    rclpy.init(args=args)
    
    # 🎯 [Single Source of Truth] 단 하나의 공유 메모리 공간 생성
    shared_blackboard = Blackboard()

    # 1. 존재하는 실물 노드들만 생성하여 동일한 shared_blackboard 주입
    web_bridge_node = WebBridgeNode(shared_blackboard)
    bt_node = AirportGuideBT(shared_blackboard)
    battery_node = BatteryNode(shared_blackboard)
    arrival_node = ArrivalNode(shared_blackboard)
    front_cam_node = FrontCameraNode(shared_blackboard)
    web_pause = WebPauseNode(shared_blackboard)


    # 2. 멀티스레드 익스큐터에 실물 노드 5개 등록
    executor = MultiThreadedExecutor()
    executor.add_node(web_bridge_node)
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    executor.add_node(arrival_node)
    executor.add_node(front_cam_node)  
    executor.add_node(web_pause) 

    try:
        # 단일 프로세스로 병렬 가동 시작
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 자원 해제
        web_bridge_node.destroy_node()
        bt_node.destroy_node()
        battery_node.destroy_node()
        arrival_node.destroy_node()
        front_cam_node.destroy_node()
        web_pause.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()