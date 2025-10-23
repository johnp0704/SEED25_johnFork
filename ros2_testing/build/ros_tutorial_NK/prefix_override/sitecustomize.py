import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nk/Desktop/SEED25/ros2_testing/install/ros_tutorial_NK'
