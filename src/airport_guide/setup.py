from setuptools import find_packages, setup

package_name = 'airport_guide'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yyj',
    maintainer_email='kus07177@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_bt = airport_guide.test_bt:main',
            'arrival_node = airport_guide.arrival_node:main',
            'front_came_node = airport_guide.front_cam_node:main',
            'battery_node = airport_guide.battery_node:main',
        ],
    },
)
