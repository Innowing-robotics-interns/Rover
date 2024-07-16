import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dstar_main')
    default_param_file = os.path.join(pkg_share, 'config', 'dwa_main.yaml')

    return LaunchDescription([
        # 定义节点
        Node(
            package='dstar_main',
            executable='dwa_main',
            name='dwa_main',
            output='screen',
            parameters=[default_param_file],
        ),
    ])