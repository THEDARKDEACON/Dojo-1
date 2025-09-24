import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robosync/Robot/camera_ws/install/nv21_converter_pkg'
