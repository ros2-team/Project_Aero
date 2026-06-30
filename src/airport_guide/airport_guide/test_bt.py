#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from airport_guide.blackboard import Blackboard, GoalState
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


class AirportGuideBT(Node):
    def __init__(self, blackboard):
        super().__init__("airport_guide_bt")
        self.blackboard = blackboard
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # [수정] _current_goal_handle을 __init__에서 미리 None으로 초기화.
        # 기존엔 _goal_response_callback이 한 번도 안 불린 시점에 cancel_nav_goal()이
        # 먼저 호출되면 hasattr() 체크로 방어하긴 했지만, 애초에 속성을 미리
        # 선언해두는 게 "이 객체가 어떤 필드를 갖는지" 명확해서 더 안전함.
        self._current_goal_handle = None

        self.pause_sub = self.create_subscription(
            Bool, '/test/pause', self._pause_callback, 10
        )

        self.root = Selector("Root")

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

        self.root.add_child(battery_br)
        self.root.add_child(sensor_br)
        self.root.add_child(pause_br)
        self.root.add_child(emergency_br)
        self.root.add_child(human_hub)
        self.root.add_child(avoid_br)
        self.root.add_child(arrival_br)
        self.root.add_child(nav_br)
        self.root.add_child(ActionIdle("SystemIdle"))

        self.timer = self.create_timer(0.1, self.bt_tick)

    def _pause_callback(self, msg):
        if msg.data:
            self.blackboard.is_paused = True
        else:
            self.blackboard.is_paused = False
            if self.blackboard.goal_state == GoalState.CANCELING:
                # [수정] 직접 대입 -> 단일 진입점 사용
                self.set_goal_state(GoalState.IDLE)

    def send_nav_goal(self, x, y):
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        # [추가] 새 goal을 보내기 전, 이전 goal의 핸들 흔적을 지워서
        # 직전 goal의 result 콜백이 늦게 도착했을 때 새 goal 상태를
        # 잘못 건드리지 않도록 방지
        self._current_goal_handle = None

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.set_goal_state(GoalState.SENT)

    def set_goal_state(self, new_state: GoalState):
        """
        goal_state를 바꾸는 유일한 진입점.
        bt_nodes.py의 어떤 Condition/Action도, 이 클래스의 어떤 콜백도
        blackboard.goal_state에 직접 대입하지 않고 반드시 이 메서드를 통해서만 바꾼다.
        """
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
        # CANCELING/IDLE 상태일 때 result가 뒤늦게 도착한 경우는
        # 이미 다른 경로(취소/복구)로 정리됐다고 보고 DONE으로 덮어쓰지 않는다.
        if self.blackboard.goal_state not in [GoalState.CANCELING, GoalState.IDLE]:
            self.set_goal_state(GoalState.DONE)
        self._current_goal_handle = None

    def cancel_nav_goal(self):
        if self.blackboard.goal_state in [GoalState.RUNNING, GoalState.SENT]:
            if self._current_goal_handle is not None:
                self.set_goal_state(GoalState.CANCELING)
                self._current_goal_handle.cancel_goal_async()
            else:
                # goal_handle이 아직 도착하기 전(accept 응답 전)인데 cancel이 들어온
                # 애매한 타이밍 -> CANCELING을 거치지 않고 바로 IDLE로 정리
                self.set_goal_state(GoalState.IDLE)

    def bt_tick(self):
        self.root.tick(self.blackboard, self)


def main(args=None):
    rclpy.init(args=args)
    shared_blackboard = Blackboard()

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