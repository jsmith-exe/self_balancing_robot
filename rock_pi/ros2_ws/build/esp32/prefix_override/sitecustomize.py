import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/js/self_balancing_robot/rock_pi/ros2_ws/install/esp32'
