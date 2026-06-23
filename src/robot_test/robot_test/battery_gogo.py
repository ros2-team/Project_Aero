import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import cv2
import numpy as np
import sys

# from nav2_msgs.action import NavigateThroughPoses

class BatteryDocking(Node):
    def __init__(self):
        # 1. 노드 이름 초기화
        super().__init__('battery_gogo')
        
        # 2. 퍼블리셔, 서브스크라이버, 액션 클라이언트 선언 등
        self.subscription = self.create_subscription(CompressedImage, '/cam2/image_raw/compressed', self.cam_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        ## 이미지 변환 bridge 클래스 선언 ##
        self.bridge = CvBridge()

        ### aruco 마커 인식에 사용할 내부 파라미터 값 가져오기 ###
        self.filepath = "/home/kim/airport_robot/src/robot_test/config/front_camera_info.yaml"
        self.fs = cv2.FileStorage(self.filepath, cv2.FILE_STORAGE_READ)

        if not self.fs.isOpened():
            print(f"❌ YAML 파일을 열 수 없습니다. 경로와 첫 줄(%YAML:1.0)을 확인하세요: {self.filepath}")
            sys.exit()
        
        # YAML 변수명에 맞게 가져오기
        self.camera_matrix = self.fs.getNode("camera_matrix").mat()
        self.dist_coeff = self.fs.getNode("dist_coeff").mat() 
        self.fs.release()

        # aruco 딕셔너리 세팅 !!!!!!
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

    # 4. 콜백 함수나 실제 동작할 메서드들 작성
    def cam_cb(self, msg):
        try:
            ##### bridge 객체로 compressed 이미지 opencv 전용으로 변환 #####
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.arucoruco(cv_image)

            cv2.imshow("ROS2 Camera Pipeline", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'Compressed 변환 실패: {e}')

        pass
    
    def arucoruco(self, data):
        cv_image = data
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i in range(len(ids)):
                marker_id = ids[i][0]

                # 3. 마커 ID에 따라 실제 크기 동적 할당 (핵심 포인트!)
                if marker_id == 10:    # 바깥쪽 큰 마커
                    current_marker_length = 0.13   # 130mm
                elif marker_id == 11:  # 안쪽 작은 마커
                    current_marker_length = 0.026  # 26mm
                else:
                    current_marker_length = 0.13   # 그 외는 일단 130mm로 간주

                # 현재 마커 크기에 맞춘 3D 모서리 좌표 생성
                obj_points = np.array([
                    [-current_marker_length / 2,  current_marker_length / 2, 0],
                    [ current_marker_length / 2,  current_marker_length / 2, 0],
                    [ current_marker_length / 2, -current_marker_length / 2, 0],
                    [-current_marker_length / 2, -current_marker_length / 2, 0]
                ], dtype=np.float32)

                # 포즈 추정 (solvePnP)
                success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], self.camera_matrix, self.dist_coeff)

                if success:
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]

                    # Yaw 각도 추출
                    rmat, _ = cv2.Rodrigues(rvec)
                    euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                    yaw = euler_angles[1] 

                    # 결과 출력
                    print(f"marker_ids: [{marker_id}] (Size: {current_marker_length*1000:.0f}mm)")
                    print("pose:")
                    print("  position:")
                    print(f"    x: {x:.3f}")
                    print(f"    y: {y:.3f}")
                    print(f"    z: {z:.3f}")
                    print("  rotation:")
                    print(f"    yaw: {yaw:.3f}")
                    print("-" * 25)

                    # 화면에 축 그리기
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeff, rvec, tvec, current_marker_length / 2)
                    print('축을 그렸습니다')
                    
def main(args=None):
    # 1. ROS2 통신 초기화
    rclpy.init(args=args)

    # 2. 노드 객체 생성
    batterydocking = BatteryDocking()

    try:
        # 3. 노드 실행 (콜백 함수들이 무한 루프로 대기)
        rclpy.spin(batterydocking)

    except KeyboardInterrupt:
        # Ctrl+C 등으로 종료 요청이 들어왔을 때의 예외 처리
        batterydocking.get_logger().info("사용자에 의해 노드가 종료됩니다.")

    finally:
        # 4. 자원 해제 및 ROS2 종료
        batterydocking.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()