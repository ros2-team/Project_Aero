import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/celestial/projectAR/ros2_ws/install/guide_robot'
