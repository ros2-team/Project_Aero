import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2
import numpy as np
import sys
import math

# 화면 중앙 정렬 기준: 이미지 가로 중앙 기준 이 비율(25%) 이내에 마커가 있어야
# "화각 안에 안정적으로 들어왔다"고 보고 solvePnP(3D 계산)를 실행하고 그 값을 신뢰함.
# BatteryDocking(계산 여부 판단)과 DockingController(전환 여부 판단) 둘 다 이 값을 참조해야
# 서로 어긋나지 않으므로 여기 한 곳에서만 정의한다.
PIXEL_CENTER_THRESHOLD = 0.25


class BatteryDocking(Node):
    def __init__(self):
        super().__init__('battery_gogo')

        self.subscription = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.cam_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_yaw = None
        self.latest_z = None
        self.latest_x = None
        self.latest_pixel_offset = None   # 마커 중심이 화면 가로 중앙에서 얼마나 벗어났는지 (-1~+1)
        self.marker_visible = False

        self.lost_count = 0
        self.marker_visible_filtered = False

        self.LOST_THRESHOLD = 10

        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.bridge = CvBridge()

        self.filepath = "/home/ksj/test_team4/src2/package/config/front_camera_info.yaml"
        self.fs = cv2.FileStorage(self.filepath, cv2.FILE_STORAGE_READ)
        if not self.fs.isOpened():
            print(f"❌ YAML 파일을 열 수 없습니다: {self.filepath}")
            sys.exit()
        self.camera_matrix = self.fs.getNode("camera_matrix").mat()
        self.dist_coeff = self.fs.getNode("dist_coeff").mat()
        self.fs.release()

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        self.docking_controller = DockingController()

    def cam_cb(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.arucoruco(cv_image)
            cv2.imshow("ROS2 Camera Pipeline", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f'Compressed 변환 실패: {e}')
            import traceback
            traceback.print_exc()

    def control_loop(self):
        self.docking_controller.control(
            self.latest_x, self.latest_z, self.latest_yaw, self.latest_pixel_offset,
            self.marker_visible, self.pub
        )

    def arucoruco(self, data):
        cv_image = data
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        found = False
        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            idx_11 = None
            idx_10 = None
            for i in range(len(ids)):
                if ids[i][0] == 11:
                    idx_11 = i
                elif ids[i][0] == 10:
                    idx_10 = i

            if idx_11 is not None:
                use_idx = idx_11
                current_marker_length = 0.026
                used_id = 11
            elif idx_10 is not None:
                use_idx = idx_10
                current_marker_length = 0.130
                used_id = 10
            else:
                use_idx = None

            if use_idx is not None:
                # 마커가 화면(픽셀) 가로 중앙에서 얼마나 벗어났는지 (-1: 왼쪽 끝, 0: 중앙, +1: 오른쪽 끝)
                marker_center_px = corners[use_idx][0].mean(axis=0)
                image_width = cv_image.shape[1]
                image_center_x = image_width / 2.0
                pixel_offset = (marker_center_px[0] - image_center_x) / image_center_x

                found = True
                self.latest_pixel_offset = pixel_offset

                obj_points = np.array([
                    [-current_marker_length / 2,  current_marker_length / 2, 0],
                    [ current_marker_length / 2,  current_marker_length / 2, 0],
                    [ current_marker_length / 2, -current_marker_length / 2, 0],
                    [-current_marker_length / 2, -current_marker_length / 2, 0]
                ], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[use_idx][0], self.camera_matrix, self.dist_coeff,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    rmat, _ = cv2.Rodrigues(rvec)
                    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                    yaw_deg = euler_angles[1]

                    self.latest_yaw = yaw_deg
                    self.latest_z = z
                    self.latest_x = x

                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeff, rvec, tvec, current_marker_length / 2)

                print(f"[사용 중인 마커: ID {used_id}] pixel_offset={pixel_offset:.2f}")

        if found:
            self.lost_count = 0
            self.marker_visible_filtered = True   # 인식되면 즉시 신뢰
        else:
            self.lost_count += 1

        if self.lost_count >= self.LOST_THRESHOLD:
            self.marker_visible_filtered = False
            self.latest_x = None
            self.latest_z = None
            self.latest_yaw = None
            self.latest_pixel_offset = None
        self.marker_visible = self.marker_visible_filtered


class DockingController:
    def __init__(self):
        self.state = "INIT_SEARCH"   # 시작: 마커 찾기
        self.last_turn_sign = 1      # 마지막 회전 방향 (재탐색용)

        self.Z_TARGET = 0.5          # 최종 도킹 거리 50cm (근거리 인식 불안정 구간 회피)
        self.X_THRESHOLD = 0.04      # x 정렬 기준 4cm
        self.YAW_THRESHOLD = 10      # yaw 정렬 기준 10도
        self.Z_SWITCH_MAX = 0.6      # 이 거리(60cm) 이내여야 x,yaw 정렬 판정을 신뢰함

        # CENTER_ALIGN: 마커를 화면(픽셀) 가로 중앙에 오도록 제자리 회전으로 맞춘 뒤에만
        # x, yaw, z 값을 신뢰해서 접근을 시작함
        # (PIXEL_CENTER_THRESHOLD는 파일 상단의 전역 상수를 참조 - BatteryDocking과 값을 통일하기 위함)
        self.KP_CENTER = 0.25                  # 픽셀 오프셋 → 회전 속도 게인 (낮춰서 과회전 방지)
        self.CENTER_ALIGN_MAX_SPEED = 0.1      # 중앙 정렬 시 최대 회전 속도 (낮춰서 과회전 방지)

        # yaw 튐 필터: 마지막으로 믿었던 값(prev_yaw)과 비교해서
        # 이번 값이 너무 많이(YAW_JUMP_THRESHOLD 이상) 다르면 일단 노이즈로 의심하고 무시하되,
        # 연속으로 YAW_CONFIRM_COUNT번 비슷하게 "다른 값"이 나오면 그건 노이즈가 아니라
        # 진짜로 yaw가 변한 것으로 보고 받아들인다.
        # (한 번이라도 튀면 영원히 그 값에 고정되어 버리는 문제를 막기 위함)
        self.prev_yaw = None
        self.YAW_JUMP_THRESHOLD = 15
        self.yaw_jump_streak = 0
        self.yaw_jump_candidate = None
        self.YAW_CONFIRM_COUNT = 3

        # APPROACH 중 x,yaw 조건을 통과하지 못한 채 계속 접근하다가
        # 너무 가까워지는 것을 막는 안전 최소 거리
        # (이 거리보다 가까워지면 정렬이 안 됐어도 강제로 멈추고 재탐색/재정렬 하도록 함)
        self.APPROACH_MIN_Z = 0.2   # 20cm

        self.LINEAR_SPEED = 0.05
        self.KP_CURVE = 1.5
        self.KP_YAW_FINE = 0.015          # x가 맞은 후 yaw만 보정할 때 쓰는 게인 (약하게)
        self.MAX_ANGULAR_SPEED = 0.2     # 0.3→0.2: 초반 x가 클 때 너무 세게 돌아서
                                          # 모션 블러로 yaw 측정이 튀는 것을 줄이기 위함
        self.MAX_YAW_FINE_SPEED = 0.05   # yaw만 보정할 때는 x보정과 분리된 낮은 상한 사용
                                          # (같은 상한을 쓰면 yaw가 클 때 결국 세게 돌아버려 x를 다시 흔듦)
        self.SEARCH_SPEED = 0.1          # 접근 중 놓쳤을 때 재탐색 속도
        self.INIT_SEARCH_SPEED = 0.15    # 처음 마커 찾기 속도 (천천히, 왼쪽)

        # x가 이보다 작으면(오버슈팅 찰나 등) last_turn_sign을 갱신하지 않음
        self.TURN_SIGN_UPDATE_THRESHOLD = 0.03

    def _clamp(self, value, max_value):
        return max(-max_value, min(max_value, value))

    def control(self, x, z, yaw_deg, pixel_offset, marker_visible, publisher):
        # ---- yaw 튐 필터링 (여기서 한 번만 처리, 아래 모든 상태가 이 값을 씀) ----
        if marker_visible and yaw_deg is not None:
            if self.prev_yaw is not None and abs(yaw_deg - self.prev_yaw) > self.YAW_JUMP_THRESHOLD:
                # 처음 튄 값이거나, 이전과 다른 방향으로 튄 값이면 후보를 새로 시작
                if self.yaw_jump_candidate is None or abs(yaw_deg - self.yaw_jump_candidate) > self.YAW_JUMP_THRESHOLD:
                    self.yaw_jump_candidate = yaw_deg
                    self.yaw_jump_streak = 1
                else:
                    # 이전 튄 값과 비슷한 값이 또 나옴 → 진짜 변화일 가능성이 쌓임
                    self.yaw_jump_streak += 1

                if self.yaw_jump_streak >= self.YAW_CONFIRM_COUNT:
                    # 연속으로 확인됨 → 노이즈가 아니라 진짜 변화로 인정
                    self.prev_yaw = yaw_deg
                    self.yaw_jump_candidate = None
                    self.yaw_jump_streak = 0
                else:
                    # 아직 확인 안 됨 → 일단 무시하고 이전 값 사용
                    yaw_deg = self.prev_yaw
            else:
                # 정상 범위 → 이 값을 새로운 기준(prev_yaw)으로 갱신, 튐 후보 리셋
                self.prev_yaw = yaw_deg
                self.yaw_jump_candidate = None
                self.yaw_jump_streak = 0

        twist = Twist()

        # ===== INIT_SEARCH: 처음에 마커 찾기 (왼쪽으로 천천히 회전) =====
        if self.state == "INIT_SEARCH":
            if marker_visible:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "CENTER_ALIGN"
                print(f"🎯 마커 발견! → 화면 중앙 정렬 시작")
                return
            twist.linear.x = 0.0
            twist.angular.z = self.INIT_SEARCH_SPEED
            publisher.publish(twist)
            print(f"🔄 마커 찾는 중... (왼쪽 회전)")
            return

        # ===== CENTER_ALIGN: 마커를 화면 가로 중앙에 오도록 제자리 회전 =====
        # 픽셀 위치가 충분히 중앙에 들어왔을 때만 x,yaw,z 값을 신뢰하고 접근 시작
        elif self.state == "CENTER_ALIGN":
            if not marker_visible or pixel_offset is None:
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                publisher.publish(stop_twist)
                self.state = "SEARCH"
                print(f"⚠️ 중앙 정렬 중 마커 놓침 → 재탐색 (last_turn_sign={self.last_turn_sign})")
                return

            if abs(pixel_offset) <= PIXEL_CENTER_THRESHOLD:
                # 화면 중앙 근처에 들어옴 → 이제 x,yaw,z 값을 믿고 접근 시작
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "APPROACH"
                print(f"✓ 화면 중앙 정렬 완료 (pixel_offset={pixel_offset:.2f}) → 접근 시작")
                return

            # 아직 중앙이 아님 → 제자리에서 회전만 (전진 없음)
            angular_z = -pixel_offset * self.KP_CENTER
            angular_z = self._clamp(angular_z, self.CENTER_ALIGN_MAX_SPEED)

            twist.linear.x = 0.0
            twist.angular.z = angular_z

            if abs(angular_z) > 0.01:
                self.last_turn_sign = 1 if angular_z > 0 else -1

            publisher.publish(twist)
            print(f"↔️ 화면 중앙 정렬 중: pixel_offset={pixel_offset:.2f} | angular={angular_z:.2f}")
            return

        # ===== APPROACH: 곡선 접근 (x 보정). x,yaw 둘 다 맞으면 FINAL_FORWARD로 전환 =====
        elif self.state == "APPROACH":
            if not marker_visible:
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                publisher.publish(stop_twist)
                self.state = "SEARCH"
                print(f"⚠️ 마커 놓침 → 재탐색 (last_turn_sign={self.last_turn_sign})")
                return

            # 안전장치: 정렬(x,yaw)이 안 끝났는데도 너무 가까워지면
            # (마커 인식이 근거리에서 왜곡/불안정해질 수 있으므로) 강제로 멈추고 재정렬
            if z <= self.APPROACH_MIN_Z:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "CENTER_ALIGN"
                print(f"⚠️ 정렬 전에 너무 가까워짐(z={z*100:.1f}cm) → 재정렬")
                return

            # x, yaw 둘 다 기준 이내 + z가 충분히 가까울 때만 → 최종 직진 단계로 전환
            # (거리가 너무 멀면 x, yaw 측정 자체가 불안정해서 판정을 믿을 수 없음)
            if abs(x) <= self.X_THRESHOLD and abs(yaw_deg) <= self.YAW_THRESHOLD and z <= self.Z_SWITCH_MAX:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "FINAL_FORWARD"
                print(f"✓ x, yaw 정렬 완료 (x={x*100:.1f}cm, yaw={yaw_deg:.1f}°) → 최종 직진 시작")
                return

            # x는 이미 맞았는데 yaw가 아직 안 맞은 경우:
            # 곡선 보정(x 기준)은 x가 0에 가까우면 회전량도 0에 가까워져서 더 이상 yaw를 고치지 못함.
            # 그러면 yaw가 큰 채로 영원히 APPROACH에 머무를 수 있으므로,
            # x는 유지하면서 yaw만 살짝 보정해주는 단계를 별도로 둔다.
            if abs(x) <= self.X_THRESHOLD:
                angular_z = -yaw_deg * self.KP_YAW_FINE
                angular_z = self._clamp(angular_z, self.MAX_YAW_FINE_SPEED)

                twist.linear.x = self.LINEAR_SPEED
                twist.angular.z = angular_z

                if abs(angular_z) > 0.01:
                    self.last_turn_sign = 1 if angular_z > 0 else -1

                publisher.publish(twist)
                print(f"↻ yaw 보정 중: z={z*100:.1f}cm | x={x*100:.1f}cm | yaw={yaw_deg:.1f}° | angular={angular_z:.2f}")
                return

            # 아직 x조차 안 맞음 → 곡선 보정 (x 기준)
            angular_z = -x * self.KP_CURVE
            angular_z = self._clamp(angular_z, self.MAX_ANGULAR_SPEED)

            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = angular_z

            if abs(x) > self.TURN_SIGN_UPDATE_THRESHOLD:
                self.last_turn_sign = 1 if angular_z > 0 else -1

            publisher.publish(twist)
            print(f"➡️ 접근 중: z={z*100:.1f}cm | x={x*100:.1f}cm | yaw={yaw_deg:.1f}° | angular={angular_z:.2f}")
            return

        # ===== FINAL_FORWARD: 회전 없이 순수 직진, z만 줄임 =====
        elif self.state == "FINAL_FORWARD":
            if not marker_visible:
                # 회전 재탐색 하지 않고 그냥 정지, 마커 다시 보이면 이어서 직진
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                publisher.publish(stop_twist)
                print(f"⚠️ 최종 직진 중 마커 놓침 → 정지 (재탐색 없음)")
                return

            if z <= self.Z_TARGET:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "DONE"
                print(f"✅ 도킹 완료! z={z*100:.1f}cm | x={x*100:.1f}cm | yaw={yaw_deg:.1f}°")
                return

            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = 0.0
            publisher.publish(twist)
            print(f"➡️ 최종 직진: z={z*100:.1f}cm | x={x*100:.1f}cm | yaw={yaw_deg:.1f}°")
            return

        # ===== SEARCH: 접근/직진 중 놓쳤을 때 재탐색 =====
        elif self.state == "SEARCH":
            if marker_visible:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "CENTER_ALIGN"
                print(f"🔍 마커 재발견 → 중앙 정렬 재확인")
                return

            twist.linear.x = 0.0
            twist.angular.z = -self.last_turn_sign * self.SEARCH_SPEED
            publisher.publish(twist)
            print(f"🔍 마커 탐색 중... (반대방향={-self.last_turn_sign})")
            return

        # ===== DONE: 정지 =====
        elif self.state == "DONE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            publisher.publish(twist)
            return


def main(args=None):
    rclpy.init(args=args)
    batterydocking = BatteryDocking()
    try:
        rclpy.spin(batterydocking)
    except KeyboardInterrupt:
        batterydocking.get_logger().info("종료됩니다.")
    finally:
        batterydocking.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()