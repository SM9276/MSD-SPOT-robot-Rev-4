import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/msd/MSD2/REV4/src/install/spotarm_assembly_description'
