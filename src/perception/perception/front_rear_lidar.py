#!/home/yeojin/yolo_test/venv/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data
import math
from ar_interfaces.msg import LidarScanData
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanLidar(Node):
    def __init__(self):
        super().__init__('front_rear_scan')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        self.pub_front = self.create_publisher(LidarScanData, '/scan/front', sensor_qos)
        self.pub_rear = self.create_publisher(LidarScanData, '/scan/rear', sensor_qos)
        self.bridge = CvBridge()
        self.lidar = []

    def lidar_cb(self, data):
    # 1. 전체 데이터 numpy로 한 번에 변환
        ranges = np.array(data.ranges)
        dist_cm = ranges * 100.0

        angles_deg = np.degrees(
            data.angle_min + np.arange(len(ranges)) * data.angle_increment
        )

        # 2. 거리 필터 마스크 (nan/inf 동시 처리)
        ### 이건 아래 조건에 따라 True 혹은 False를 반환함
        valid = (dist_cm > 0.0) & (dist_cm <= 200.0)

        # 3. 전방/후방 마스크
        ### 전 후방 조건 다르게 하여 저장
        front_mask = valid & ((angles_deg <= 90.0) | (angles_deg >= 270.0))
        rear_mask  = valid & (angles_deg > 90.0) & (angles_deg < 270.0)

        # 4. 각도 매핑
        mapped_all = np.where(angles_deg <= 90.0,
                            90.0 - angles_deg,    # 전방 좌측
                            450.0 - angles_deg)   # 전방 우측 (270~360)
        mapped_rear = 270.0 - angles_deg

        # 5. 마스크 적용 + 정렬
        ### 마스크 적용하여 true 인 경우에만 해당 인덱스 값 저장함
        f_angles = mapped_all[front_mask]
        f_dists  = dist_cm[front_mask]
        f_idx    = np.argsort(f_angles)
        f_angles, f_dists = f_angles[f_idx], f_dists[f_idx]

        r_angles = mapped_rear[rear_mask]
        r_dists  = dist_cm[rear_mask]
        r_idx    = np.argsort(r_angles)
        r_angles, r_dists = r_angles[r_idx], r_dists[r_idx]

        self.front_pub(f_angles, f_dists)
        self.rear_pub(r_angles, r_dists)

    def rear_pub(self, r_angles, r_dists):
        """ 후방 라이다 각도 및 거리 값 pub """
        msg = LidarScanData()
        msg.angles = r_angles.tolist()
        msg.distances = r_dists.tolist()   # numpy → list 변환은 여기서만
        self.pub_rear.publish(msg)

    def front_pub(self, f_angles, f_dists):
        """ 전방 라이다 각도 및 거리 값 pub """
        msg = LidarScanData()
        msg.angles = f_angles.tolist()
        msg.distances = f_dists.tolist()   # numpy → list 변환은 여기서만
        self.pub_front.publish(msg)
        
def main(args=None):
    # 1. ROS2 통신 초기화
    rclpy.init(args=args)

    # 2. 노드 객체 생성
    sl = ScanLidar()

    try:
        # 3. 노드 실행 (콜백 함수들이 무한 루프로 대기)
        rclpy.spin(sl)

    except KeyboardInterrupt:
        # Ctrl+C 등으로 종료 요청이 들어왔을 때의 예외 처리
        pass

    finally:
        # 4. 자원 해제 및 ROS2 종료
        sl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()