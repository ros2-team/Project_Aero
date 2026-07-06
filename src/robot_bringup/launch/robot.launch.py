from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('perception'),
                'launch',
                'perception.launch.py'
            )
        )
    )

    behavior_tree_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('behavior_tree'),
                'launch',
                'behavior_tree.launch.py'
            )
        )
    )

    web_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('web_bridge'),
                'launch',
                'web_bridge.launch.py'
            )
        )
    )

    return LaunchDescription([
        perception_launch,
        behavior_tree_launch,
        web_bridge_launch,
    ])