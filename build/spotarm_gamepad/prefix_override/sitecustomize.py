import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/Oden2/Downloads/MSD-SPOT-robot-Rev-4/install/spotarm_gamepad'
