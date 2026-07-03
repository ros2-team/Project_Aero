from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 후방 카메라 노드
    # 파일명은 rearcam.py 이지만, setup.py에 'rearcam'으로 등록됨
    rearcam_node = Node(
        package='robot_test',
        executable='rearcam',
        name='rearcam_node',
        output='screen'
    )

    # 2. 후방 카메라 제어 노드
    # 파일명은 rear_cam_ctrl.py 이지만, setup.py에 'rearcam_ctrl'로 등록됨
    rearcam_ctrl_node = Node(
        package='robot_test',
        executable='rearcam_ctrl',
        name='rearcam_ctrl_node',
        output='screen'
    )

    # 3. 전/후방 라이다 노드
    # 파일명은 front_rear_lidar.py 이지만, setup.py에 'frontrear_lidar'로 등록됨
    front_rear_lidar_node = Node(
        package='robot_test',
        executable='front_rear_lidar',
        name='front_rear_lidar_node',
        output='screen'
    )

    # 모든 노드를 묶어서 반환
    return LaunchDescription([
        rearcam_node,
        rearcam_ctrl_node,
        front_rear_lidar_node
    ])