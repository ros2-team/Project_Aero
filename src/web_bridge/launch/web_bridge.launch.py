from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    web_bridge_node = Node(
        package='web_bridge',
        executable='web_data',
        name='web_bridge_node',
        output='screen'
    )

    return LaunchDescription([
        web_bridge_node,
    ])