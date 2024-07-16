import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    dwa_local_planner_path = get_package_share_directory('dwa_main')
    # 构建配置文件的完整路径
    config_file_path = os.path.join(dwa_local_planner_path, 'config', 'dwa_local_planner_params.yaml')

    # 使用LaunchConfiguration引用配置文件路径
    config_file = LaunchConfiguration('config_file')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,  # 直接提供默认路径字符串
        description='Full path to the ROS2 parameters file to use for the node')

    dwa_local_planner_node = Node(
        package='dwa_main',
        executable='dwa_main',
        name='dwa_local_planner',
        output='screen',
        parameters=[config_file]  # 使用LaunchConfiguration引用
    )

    ld = LaunchDescription()

    ld.add_action(declare_config_file_cmd)
    ld.add_action(dwa_local_planner_node)

    return ld