import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vighneshreddy/MRM_VighneshReddy/install/robot_position'
