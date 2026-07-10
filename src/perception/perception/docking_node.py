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


class BatteryDocking(Node):
    def __init__(self):
        super().__init__('battery_gogo')

        self.subscription = self.create_subscription(CompressedImage, '/cam1/image_raw/compressed', self.cam_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_yaw = None
        self.latest_z = None
        self.latest_x = None
        self.marker_visible = False

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

    def control_loop(self):
        # INIT_SEARCH는 마커 없이도 돌아야 하므로 None 체크 제거
        self.docking_controller.control(
            self.latest_x, self.latest_z, self.latest_yaw,
            self.marker_visible, self.pub
        )

    def arucoruco(self, data):
        cv_image = data
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        found = False
        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            # ID 11과 ID 10 각각의 인덱스 찾기
            idx_11 = None
            idx_10 = None
            for i in range(len(ids)):
                if ids[i][0] == 11:
                    idx_11 = i
                elif ids[i][0] == 10:
                    idx_10 = i

            # 우선순위: ID 11 있으면 11 사용, 없으면 10 사용
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

                    found = True
                    self.latest_yaw = yaw_deg
                    self.latest_z = z
                    self.latest_x = x

                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeff, rvec, tvec, current_marker_length / 2)
                    print(f"[사용 중인 마커: ID {used_id}]")  # 확인용 로그

        self.marker_visible = found


class DockingController:
    def __init__(self):
        self.state = "INIT_SEARCH"   # 시작: 마커 찾기
        self.last_turn_sign = 1

        self.Z_TARGET = 0.1
        self.X_THRESHOLD = 0.02

        self.LINEAR_SPEED = 0.05
        self.KP_CURVE = 1.5
        self.MAX_ANGULAR_SPEED = 0.3
        self.SEARCH_SPEED = 0.1          # 접근 중 놓쳤을 때 재탐색 속도
        self.INIT_SEARCH_SPEED = 0.15    # 처음 마커 찾기 속도 (천천히, 왼쪽)

    def _clamp(self, value, max_value):
        return max(-max_value, min(max_value, value))

    def control(self, x, z, yaw_deg, marker_visible, publisher):
        twist = Twist()

        # ===== INIT_SEARCH: 처음에 마커 찾기 (왼쪽으로 천천히 회전) =====
        if self.state == "INIT_SEARCH":
            if marker_visible:
                # 마커 발견 → 멈추고 접근 시작
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "APPROACH"
                print(f"🎯 마커 발견! → 접근 시작")
                return
            # 아직 못 찾음 → 왼쪽으로 천천히 계속 회전
            twist.linear.x = 0.0
            twist.angular.z = self.INIT_SEARCH_SPEED
            publisher.publish(twist)
            print(f"🔄 마커 찾는 중... (왼쪽 회전)")
            return

        # ===== APPROACH: 곡선 접근 (전진 + x 보정) =====
        elif self.state == "APPROACH":
            if not marker_visible:
                self.state = "SEARCH"
                print(f"⚠️ 마커 놓침 → 재탐색")
                return

            if z <= self.Z_TARGET:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "DONE"
                print(f"✅ 도킹 완료! z={z*100:.1f}cm")
                return

            angular_z = -x * self.KP_CURVE
            angular_z = self._clamp(angular_z, self.MAX_ANGULAR_SPEED)

            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = angular_z

            if abs(angular_z) > 0.01:
                self.last_turn_sign = 1 if angular_z > 0 else -1

            publisher.publish(twist)
            print(f"➡️ 접근 중: z={z*100:.1f}cm | x={x*100:.1f}cm | angular={angular_z:.2f}")
            return

        # ===== SEARCH: 접근 중 놓쳤을 때 재탐색 =====
        elif self.state == "SEARCH":
            if marker_visible:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                publisher.publish(twist)
                self.state = "APPROACH"
                print(f"🔍 마커 재발견 → 접근 재개")
                return

            twist.linear.x = 0.0
            twist.angular.z = -self.last_turn_sign * self.SEARCH_SPEED
            publisher.publish(twist)
            print(f"🔍 마커 탐색 중...")
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