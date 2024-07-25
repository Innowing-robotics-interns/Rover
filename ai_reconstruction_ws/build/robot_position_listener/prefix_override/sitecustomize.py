import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/nova_ssd/workspaces/ai_reconstruction_ws/install/robot_position_listener'
