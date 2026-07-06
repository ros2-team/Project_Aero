#!/usr/bin/env python3
import rclpy
import json
import time
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
        
        # 🔄 [원복 완료] bt_nodes에서 복잡한 통신 자원을 제거하고 메인 노드가 온전히 독점하도록 롤백했습니다.
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 🔄 [원복 완료] bt_nodes 내에 결합되어 있던 비동기 액션 서버용 상태 추적 핸들러를 메인 클래스 멤버로 복원했습니다.
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
        self.root.add_child(nav_br)          # 8순위: 모든 예외가 없을 때 자율주행 실행
        self.root.add_child(battery_br)      # 1순위: 배터리 방전 체크 
        self.root.add_child(sensor_br)       # 2순위: 센서 통신 끊김 체크
        self.root.add_child(emergency_br)    # 3순위: 전방 급정거/복구 체크
        self.root.add_child(pause_br)        # 4순위: 웹 일시정지 체크
        self.root.add_child(human_hub)       # 5순위: 사람 유실/멀어짐 체크
        self.root.add_child(avoid_br)        # 6순위: 장애물 회피 체크
        self.root.add_child(arrival_br)      # 7순위: 목적지 도착 세션 체크
        self.root.add_child(ActionIdle("SystemIdle")) # 9순위: 정말 아무것도 안 할 때의 대기

        # 웹 상태 변경 감지 및 주기 제어를 위한 변수 초기화
        self.last_robot_status_pub_time = 0.0
        self.last_nav_status = None
        self.last_nav_current_index = None
        self.last_nav_is_paused = None
        self.last_nav_route_len = 0

        self.timer = self.create_timer(0.1, self.bt_tick)

    def _pause_callback(self, msg):
        if msg.data:
            self.blackboard.is_paused = True
        else:
            self.blackboard.is_paused = False
            if self.blackboard.goal_state == GoalState.CANCELING:
                self.set_goal_state(GoalState.IDLE)


    # =====================================================================
    # 🔄 [원복 완료 지점] bt_nodes 레이어로부터 회수하여 메인 트리로 원상복구시킨 제어 로직들
    # =====================================================================

    def send_nav_goal(self, x, y):
        """ 🔄 [원복] Goal 메시지 생성 및 Nav2 액션 비동기 송신 인터페이스를 다시 메인 노드로 이관했습니다. """
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")

        # 🛠️ [핵심 안전장치] 새 명령을 내리기 전, 기존 주행 핸들과 콜백 예약을 완전히 초기화합니다.
        if hasattr(self, '_current_goal_handle') and self._current_goal_handle is not None:
            # 기존 세션이 남아있다면 잔여 콜백 간섭을 막기 위해 제거
            self._current_goal_handle = None 

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self._current_goal_handle = None

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        
        # 🔄 [원복] 비동기 응답 타깃 메서드를 다시 메인 노드의 콜백 함수로 연동했습니다.
        send_goal_future.add_done_callback(self._goal_response_callback)

        # 액션 요청을 쏘자마자 즉시 SENT 상태로 명시 변경하여 0.1초 뒤 트리의 연속 호출 현상을 차단합니다.
        self.set_goal_state(GoalState.SENT)

    def set_goal_state(self, new_state: GoalState):
        """ 🔄 [원복] 블랙보드 내부 FSM 전이 권한 및 상태 변경 공통 메서드를 메인 트리 본체로 복원했습니다. """
        old_state = self.blackboard.goal_state
        self.blackboard.goal_state = new_state
        self.get_logger().info(f"🔁 [FSM] goal_state: {old_state.name} -> {new_state.name}")

    def _goal_response_callback(self, future):
        """ 🔄 [원복] Nav2 서버의 수락 응답 패킷을 처리하고 정식 RUNNING으로 진입시키는 핵심 콜백입니다. """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2가 goal을 거절했습니다.")
            self.set_goal_state(GoalState.IDLE)
            return

        self._current_goal_handle = goal_handle
        
        # 하부 하드웨어가 명령을 공식 수락한 그 물리 시점에 철저하게 RUNNING 상태로 동기화합니다.
        self.set_goal_state(GoalState.RUNNING)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """ 🔄 [원복] 내비게이션 세션의 완료 상태를 감지하여 주행이 완결(DONE)되었음을 마킹하는 최종 콜백입니다. """
        # 오직 정상 주행 중(RUNNING)일 때 들어온 완료 신호만 인정합니다.
        # SENT, IDLE 상태 등 목적지 전환 직후에 들어오는 과거 유령 신호를 완벽히 차단합니다.
        if self.blackboard.goal_state == GoalState.RUNNING:
            self.set_goal_state(GoalState.DONE)
        else:
            self.get_logger().warn(
                f"⚠️ [FSM 가드] RUNNING이 아닌 상태({self.blackboard.goal_state.name})에서 "
                f"과거 세션 결과가 유입되어 무시 처리했습니다."
            )
        self._current_goal_handle = None

    def cancel_nav_goal(self):
        """ 🔄 [원복] 주행 강제 취소 액션 인터페이스를 다시 메인 본체로 롤백했습니다. """
        if self.blackboard.goal_state in [GoalState.RUNNING, GoalState.SENT]:
            if self._current_goal_handle is not None:
                self.set_goal_state(GoalState.CANCELING)
                self._current_goal_handle.cancel_goal_async()
            else:
                self.set_goal_state(GoalState.IDLE)


    # *********** 디버깅 및 웹 상태 보고 코드 **********************
    def bt_tick(self):
        self.get_logger().info(
            f"🔄 [BT TICK] 현재 goal_name: '{self.blackboard.goal_name}', "
            f"state: {self.blackboard.goal_state.name if self.blackboard.goal_state else 'None'}", 
            throttle_duration_sec=1.0
        )

        current_time = time.time()

        # ---------------------------------------------------------------------
        # [1] robot_status_state 발행 (1초 주기 발행)
        # ---------------------------------------------------------------------
        if current_time - self.last_robot_status_pub_time >= 1.0:
            try:
                robot_status_payload = {
                    "battery": int(getattr(self.blackboard, "battery_level", 100.0)),
                    "x": float(getattr(self.blackboard, "current_x", 0.0)),
                    "y": float(getattr(self.blackboard, "current_y", 0.0)),
                    "yaw": float(getattr(self.blackboard, "current_yaw", 0.0)),
                    "robot_status": self.blackboard.goal_state.name.lower() if self.blackboard.goal_state else "unknown",
                    "network": "disconnected" if getattr(self.blackboard, "sensor_timeout", False) else "connected"
                }
                
                msg_status = String()
                msg_status.data = json.dumps(robot_status_payload)
                self.bt_status_pub.publish(msg_status)
                
                self.last_robot_status_pub_time = current_time
                
            except Exception as e:
                self.get_logger().error(f"robot_status_state 발행 실패: {e}", throttle_duration_sec=3.0)


        # ---------------------------------------------------------------------
        # [2] navigation_state 발행 (상태 변경 시 발행)
        # ---------------------------------------------------------------------
        try:
            current_nav_status = self.blackboard.goal_state.name.lower() if self.blackboard.goal_state else "idle"
            current_route = getattr(self.blackboard, "web_route_list", [])
            current_is_paused = getattr(self.blackboard, "is_paused", False)
            current_index = getattr(self.blackboard, "current_waypoint_index", 0) 

            is_nav_changed = (
                current_nav_status != self.last_nav_status or
                current_index != self.last_nav_current_index or
                current_is_paused != self.last_nav_is_paused or
                len(current_route) != self.last_nav_route_len
            )

            if is_nav_changed:
                navigation_payload = {
                    "status": current_nav_status,
                    "type": getattr(self.blackboard, "web_action", None),
                    "route": current_route,
                    "current_index": current_index,
                    "is_paused": current_is_paused
                }
                
                msg_nav = String()
                msg_nav.data = json.dumps(navigation_payload)
                self.bt_status_pub.publish(msg_nav)
                
                self.get_logger().info(f"📢 [웹 피드백] navigation_state 변경 발송 -> status: {current_nav_status}")
                
                self.last_nav_status = current_nav_status
                self.last_nav_current_index = current_index
                self.last_nav_is_paused = current_is_paused
                self.last_nav_route_len = len(current_route)

        except Exception as e:
            self.get_logger().error(f"navigation_state 발행 실패: {e}", throttle_duration_sec=3.0)

        # ---------------------------------------------------------------------
        # 실제 행동트리 실행 (얇아진 ActionMoveToGoal 노드가 메인의 원복된 함수들을 안전하게 교차 타깃 호출)
        # ---------------------------------------------------------------------
        self.root.tick(self.blackboard, self)


def main(args=None):
    rclpy.init(args=args)
    
    shared_blackboard = Blackboard()

    web_bridge_node = WebBridgeNode(shared_blackboard)
    bt_node = AirportGuideBT(shared_blackboard)
    battery_node = BatteryNode(shared_blackboard)
    arrival_node = ArrivalNode(shared_blackboard)
    front_cam_node = FrontCameraNode(shared_blackboard)
    web_pause = WebPauseNode(shared_blackboard)

    executor = MultiThreadedExecutor()
    executor.add_node(web_bridge_node)
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    executor.add_node(arrival_node)
    executor.add_node(front_cam_node)  
    executor.add_node(web_pause) 

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        web_bridge_node.destroy_node()
        bt_node.destroy_node()
        battery_node.destroy_node()
        arrival_node.destroy_node()
        front_cam_node.destroy_node()
        web_pause.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()