import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/airlab/seed25/ros_packages/seed_controller_ws/src/ml_red_controller/install/ml_red_controller'
