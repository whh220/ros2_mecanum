import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jjx/test1/ros2_mecanum/install/sam_bot_nav'
