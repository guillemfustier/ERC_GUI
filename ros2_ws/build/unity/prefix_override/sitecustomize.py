import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alemany/ERC_GUI/ros2_ws/install/unity'
