from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.yaml')) + glob(os.path.join('map', '*.pgm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_test = robot_test.robot_test:main',
            'battery_gogo = robot_test.battery_gogo:main',
            'send_x_y = robot_test.send_x_y:main',
            'rearcam_recognizer = robot_test.rearcam_recognizer:main',
            'waypoint_test = robot_test.waypoint_test:main',
            'rearcam = robot_test.rearcam:main',
            'rearcam_ctrl = robot_test.rear_cam_ctrl:main'
        ],
    },
)
