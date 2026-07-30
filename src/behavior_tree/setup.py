from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='ehrud2235@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'arrival_node = behavior_tree.arrival_node:main',
        'battery_node = behavior_tree.battery_node:main',
        'front_cam_node = behavior_tree.front_cam_node:main',
        'path_node = behavior_tree.path_node:main',
        'bt_main = behavior_tree.bt_main:main',
        'web_pause = behavior_tree.web_pause:main',
        ],
    },
)
