import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('dstar_main')
    default_param_file = os.path.join(pkg_share, 'config', 'map2grid.yaml')

    return LaunchDescription([
        # 定义节点
        Node(
            package='dstar_main',
            executable='map2grid',
            name='map2grid',
            output='screen',
            parameters=[default_param_file],
        ),
    ])