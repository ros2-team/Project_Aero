from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),

    (
        os.path.join('share', package_name, 'templates'),
        glob('templates/*.html')
    ),

    (
        os.path.join('share', package_name, 'static'),
        glob('static/*')
    ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksj',
    maintainer_email='ksj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'web_server = my_package.web_server:main',
        ],
    },
)
