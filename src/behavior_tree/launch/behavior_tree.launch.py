from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    path_node = Node(
        package='behavior_tree',
        executable='path_node',
        name='path_node',
        output='screen'
    )

    test_bt_node = Node(
        package='behavior_tree',
        executable='test_bt',
        name='test_bt',
        output='screen'
    )

    return LaunchDescription([
        path_node,
        test_bt_node,
    ])