import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/nova_ssd/workspaces/pcl_ws/install/livox_tf'
