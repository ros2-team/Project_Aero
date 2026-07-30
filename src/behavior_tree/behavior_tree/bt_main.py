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
    ActionGlobalRecovery,
    ConditionHumanFar, ActionSignalToHuman,
    ConditionArrived, ActionStopGuide,
    ConditionHasGoal, ActionMoveToGoal,
    ConditionQrAvailable, ActionExecuteQrCall, # qr
    ActionIdle,
)
from airport_guide.battery_node import BatteryNode
from airport_guide.arrival_node import ArrivalNode
from airport_guide.front_cam_node import FrontCameraNode
from airport_guide.web_pause import WebPauseNode
from airport_guide.qr_node import QrCallNode


class AirportGuideBT(Node):
    def __init__(self, blackboard):
        super().__init__("airport_guide_bt")
        self.blackboard = blackboard
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_handle = None
        
        self._send_goal_future = None
        self._get_result_future = None
        
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
        stop_seq = Sequence("EmergencyStopSeq")
        stop_seq.add_child(ConditionEmergency("Emergency"))
        stop_seq.add_child(ActionEmergencyStop("Stop"))
        far_seq = Sequence("HumanFarSeq")
        far_seq.add_child(ConditionHumanFar("HumanFar"))
        far_seq.add_child(ActionSignalToHuman("SignalToHuman"))
        arrival_br = Sequence("ArrivalBranch")
        arrival_br.add_child(ConditionArrived("Arrived"))
        arrival_br.add_child(ActionStopGuide("StopGuide"))

        # 여기서 CANCELING 족쇄를 풀고 IDLE로 만들어 줍니다.
        recovery_br = ActionGlobalRecovery("GlobalRecovery")

        nav_br = Sequence("NavigationBranch")
        nav_br.add_child(ConditionHasGoal("HasGoal"))
        nav_br.add_child(ActionMoveToGoal("MoveToGoal"))

        qr_br = Sequence("QrBranch")
        qr_br.add_child(ConditionQrAvailable("QrAvailable"))
        qr_br.add_child(ActionExecuteQrCall("ExecuteQrCall"))

        # 최상위 Selector 자식 순서 배치
        self.root.add_child(battery_br)      # 1순위: 배터리 방전 체크 
        self.root.add_child(sensor_br)       # 2순위: 센서 통신 끊김 체크
        self.root.add_child(pause_br)        # 3순위: 웹 일시정지 체크

        self.root.add_child(far_seq)         # 4순위: 사람 유실/멀어짐 체크
        self.root.add_child(arrival_br)      # 5순위: 목적지 도착 세션 체크
        self.root.add_child(recovery_br)     # 6순위: 다 뚫었어? 그럼 멈춰있던 거 풀어줄게! (출발 준비)
        self.root.add_child(nav_br)          # 7순위: nav
        self.root.add_child(qr_br)           # 8순위: 모든 예외가 없을 때 자율주행 실행
        self.root.add_child(ActionIdle("SystemIdle")) # 9순위: 정말 아무것도 안 할 때의 대기

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

    def send_nav_goal(self, x, y):
        self.get_logger().info(f"🎯 Nav2 액션 목표 전송 시작: ({x}, {y})")

        if hasattr(self, '_send_goal_future') and self._send_goal_future is not None:
            self._send_goal_future.cancel()
            self._send_goal_future = None
            
        if hasattr(self, '_get_result_future') and self._get_result_future is not None:
            self._get_result_future.cancel()
            self._get_result_future = None

        if hasattr(self, '_current_goal_handle') and self._current_goal_handle is not None:
            self._current_goal_handle = None 

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        self.set_goal_state(GoalState.SENT)

    def set_goal_state(self, new_state: GoalState):
        old_state = self.blackboard.goal_state
        self.blackboard.goal_state = new_state
        self.get_logger().info(f"🔁 [FSM] goal_state: {old_state.name} -> {new_state.name}")

    def _goal_response_callback(self, future):
        if self.blackboard.goal_state != GoalState.SENT:
            self.get_logger().warn("⚠️ [FSM 가드] SENT 상태가 아닐 때 유입된 목표 수락 응답이므로 폐기 처리합니다.")
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2가 goal을 거절했습니다.")
            self.set_goal_state(GoalState.IDLE)
            return

        self.get_logger().info("✅ Nav2 서버가 목표를 최종 수락했습니다. 로봇 주행 전이를 시작합니다.")
        self._current_goal_handle = goal_handle
        self.set_goal_state(GoalState.RUNNING)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """ 🔄 [3단계: 내비게이션 세션 자원 관리 콜백] """
        if self.blackboard.goal_state != GoalState.RUNNING:
            return

        try:
            # [필수 검증] 수신된 실제 액션 상태 결과 추출
            action_result = future.result()
            action_status = action_result.status
            self.get_logger().info(f"📊 [디버그 계측] Nav2 액션 서버 복귀 Status 코드: {action_status}")

            if action_status == 4: 
                # Nav2 물리 도달 성공 시 DONE으로 바로 밀지 않습니다.
                # ArrivalNode가 남은 거리 계측 및 5초 대기를 보장할 수 있도록 로그만 출력하고 상태 제어권을 양보합니다.
                self.get_logger().info(f"🏁 [Nav2 도착 성공] 하부 주행 도달 완료. ArrivalNode의 정밀 도달/대기 판정을 기다립니다.")

                self.set_goal_state(GoalState.DONE)

            else:
                # 실패나 취소 시에만 복구 동작 유도를 위해 IDLE 전이
                self.get_logger().error(f"주행이 성공하지 못했습니다. (Status: {action_status})")
                self.set_goal_state(GoalState.IDLE)

        except Exception as e:
            self.get_logger().error(f"결과 상태 파싱 중 치명적 예외 발생: {e}")
            self.set_goal_state(GoalState.IDLE)

        finally:
            # 주행 완료 혹은 오판 무시 시점 이후, 
            # 인스턴스 전용 핸들링 포인터를 리셋하여 메모리를 클리어합니다.
            self._get_result_future = None
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
                    "x": float(getattr(getattr(self.blackboard, "current_x", 0.0), "current_x", 0.0) if hasattr(self.blackboard, "current_x") else 0.0),
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

        try:
            # 웹에 보여줄 navigation 상태
            if getattr(self.blackboard, "navigation_finished", False):
                current_nav_status = "done"
            # 🛠️ [핵심 수정 3] 목적지가 있더라도 일시정지 플래그가 켜져 있으면 paused를 최우선으로 내보냅니다.
            elif getattr(self.blackboard, "is_paused", False):
                current_nav_status = "paused"
            elif getattr(self.blackboard, "goal_name", "") != "":
                current_nav_status = "moving"
            else:
                current_nav_status = "idle"

            current_route = getattr(self.blackboard, "web_route_list", [])
            current_is_paused = getattr(self.blackboard, "is_paused", False)
            current_index = getattr(self.blackboard, "current_waypoint_index", 0) 
            current_goal_name = getattr(self.blackboard, "goal_name", "")

            should_publish_navigation = False

            # 최초
            if self.last_nav_status is None:
                should_publish_navigation = True

            # ⭐ 가장 중요
            elif current_index != self.last_nav_current_index:
                should_publish_navigation = True

            # pause
            elif current_is_paused != self.last_nav_is_paused:
                should_publish_navigation = True

            # route 변경
            elif len(current_route) != self.last_nav_route_len:
                should_publish_navigation = True

            # 최종 완료만 done 전송
            elif (
                current_nav_status == "done"
                and getattr(self.blackboard, "navigation_finished", False)
                and self.last_nav_status != "done"
            ):
                should_publish_navigation = True

            if should_publish_navigation:
                # 관제 브릿지 파싱 에러 방지를 위해 원본 스키마("status", "route", "current_target") 엄격 준수
                navigation_payload = {
                    "status": current_nav_status,
                    "type": getattr(self.blackboard, "web_action", None),
                    "current_target": current_goal_name,  # 실시간 파싱 추적용 보완 자원 추가
                    "route": current_route,               # 원본 경로 보존 및 실시간 웹 매핑 지원
                    "current_index": current_index,
                    "is_paused": current_is_paused,
                    "navigation_active": getattr(self.blackboard, "navigation_active", False),
                    "navigation_finished": getattr(self.blackboard, "navigation_finished", False)
                }
                
                msg_nav = String()
                msg_nav.data = json.dumps(navigation_payload)
                self.bt_status_pub.publish(msg_nav)
                
                self.get_logger().info(
                    f"📢 [웹 피드백] navigation_state 변경 발송 -> status: {current_nav_status}, "
                    f"타깃명: {current_goal_name}, Index: {current_index}"
                )
                
                self.last_nav_status = current_nav_status
                self.last_nav_current_index = current_index
                self.last_nav_is_paused = current_is_paused
                self.last_nav_route_len = len(current_route)

        except Exception as e:
            self.get_logger().error(f"navigation_state 발행 실패: {e}", throttle_duration_sec=3.0)

        # ---------------------------------------------------------------------
        # 실제 행동트리 실행
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
    qr_node = QrCallNode(shared_blackboard)

    executor = MultiThreadedExecutor()
    executor.add_node(web_bridge_node)
    executor.add_node(bt_node)
    executor.add_node(battery_node)
    executor.add_node(arrival_node)
    executor.add_node(front_cam_node)  
    executor.add_node(web_pause) 
    executor.add_node(qr_node) 

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
        qr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()