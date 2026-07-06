from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 행동 트리 및 메인 제어 노드 (MultiThreadedExecutor 포함)
        Node(
            package='airport_guide',
            executable='test_bt',     # setup.py에 등록된 실행 이름
            name='test_bt_node',
            output='screen',          # 터미널에 로그를 띄우기 위해 필수!
            emulate_tty=True          # 로그 색상 깨짐 방지
        ),
        
        # 2. 방금 우리가 분리해낸 튼튼한 독립형 경로 전처리 노드
        Node(
            package='airport_guide',
            executable='path_node',   
            name='path_node',
            output='screen',
            emulate_tty=True
        ),
        
        # 3. 웹 통신 브릿지 노드
        Node(
            package='airport_guide',
            executable='web_data',
            name='web_data_node',
            output='screen',
            emulate_tty=True
        )
    ])