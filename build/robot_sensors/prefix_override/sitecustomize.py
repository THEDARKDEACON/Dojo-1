import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gareth-joel/Documents/gifted_hodgkin_x11/Dojo/install/robot_sensors'
