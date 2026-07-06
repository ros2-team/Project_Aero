import numpy as np
import rclpy
from rclpy.node import Node
import cv2
from rclpy.qos import qos_profile_sensor_data
import math
from robot_test_msgs.msg import LidarScanData
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        ### 타겟 바운딩 박스 xmin, centerx, xmax 구독
        self.box_sub = self.create_subscription(Float32MultiArray, '/target/bounding_box_x', self.box_callback, 10)

        ### 라이다 스캔 데이터 구독
        self.scan_sub = self.create_subscription(LidarScanData, '/scan/rear', self.lidar_callback, qos_profile_sensor_data)

        ### cmd_vel 값 publish
        self.human_status_pub = self.create_publisher(Bool, "/perception/human_far", 10)

        ### timer콜백함수 정의
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.xmin = None
        self.xmax = None
        self.angles = None
        self.distances = None

        self.get_logger().info(" 후방 카메라 가동!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    ### 바운딩 박스의 xmin, xmax, center x값 저장
    def box_callback(self, data):
        if data is None:
            pass
        self.xmin = data.data[0]
        cx = data.data[1]
        self.xmax = data.data[2]
        
        print(f"x min값: {self.xmin:.1f}, x max값: {self.xmax:.1f}")

    ### 라이다 각도와 거리 데이터 저장
    def lidar_callback(self, data):
        if data is not None:
            self.angles = np.array(data.angles)
            self.distances = np.array(data.distances)
            # print(f"각도: {angles}, 거리: {distances}")
        else:
            pass

    ### xmin, xmax YOLO 바운딩 박스의 좌우 x좌표 
    ### angles, distances 전처리된 라이다 각도, 거리
    def process_fusion_data(self):
        ### Bounding Box 좌표를 라이다 각도로 변환
        ### theta(x) = 0.25 * x + 20
        angle_min_limit = 0.25 * self.xmin + 20.0
        angle_max_limit = 0.25 * self.xmax + 20.0

        ### Numpy 마스킹: 해당 각도 범위 안에 있는 데이터만 True로 필터링
        ### 조건: angle_min_limit <= angles_deg <= angle_max_limit
        mask = (self.angles >= angle_min_limit) & (self.angles <= angle_max_limit)

        ### 마스크를 적용하여 범위 내의 거리 데이터만 추출
        target_distances = self.distances[mask]
        target_angles = self.angles[mask] # 필요시 확인용으로 추출

        ### 결과 도출
        if len(target_distances) > 0:
            ### 타겟 영역 안에 200cm 이하의 물체가 존재함! -> 최솟값 추출
            min_distance_cm = np.min(target_distances)
            
            ### 최솟값을 가진 데이터의 실제 각도가 궁금하다면 (디버깅용)
            min_idx = np.argmin(target_distances)
            min_angle_deg = target_angles[min_idx]
            
            # print(f"[장애물 감지] 픽셀({self.xmin}~{self.xmax}) -> 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역")
            # print(f"   가장 가까운 장애물: {min_distance_cm:.1f}cm (위치: {min_angle_deg:.1f}°)")
            
            return min_distance_cm
        else:
            ### Bounding Box 영역 안에 있긴 하지만, 거리가 200cm 밖이라 
            ### 전처리 과정에서 날아갔거나 아예 텅 빈 허공인 경우
            # print(f" [안전] 각도({angle_min_limit:.1f}°~{angle_max_limit:.1f}°) 영역 내 200cm 이하 장애물 없음")
            
            return None ### 혹은 안전을 뜻하는 기본값 반환 (예: 200.0)
        
    def timer_callback(self):
        ### 예외처리 처음 시작시 데이터가 없을 때
        if (self.xmin is None) or (self.angles is None):
            return
        min_dist = self.process_fusion_data()
        
        ### 융합된 결과물(min_dist)을 이용해 로봇 제어 명령 퍼블리시
        if min_dist is not None:
            if min_dist > 120.0:
                ### 사용자가 120cm이상 너무 멀어지면
                self.get_logger().warn(f" 사용자 멀어짐 {min_dist:.1f}cm" )

                self.human_far = True
                self.publish_human_status(True)
                # ### 전후진(x)과 회전(z) 속도를 0으로 설정하여 멈춤
                # self.cmd_pub()

            else:
                ### 적정 거리 유지 중
                self.get_logger().info(f" 적정 거리 유지 중 {min_dist:.1f}cm")
                self.human_far = False
        else:
            pass

    def publish_human_status(self, is_far: bool):
        msg = Bool()
        msg.data = is_far
        self.human_status_pub.publish(msg)
        
        # 값이 잘 날아가는지 터미널에서 확인하기 위한 로그
        if is_far:
            self.get_logger().info("📤 [Pub] human_lost 토픽 발행: True (정지 요청)")
        else:
            self.get_logger().info("📤 [Pub] human_lost 토픽 발행: False (추종 계속)")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()