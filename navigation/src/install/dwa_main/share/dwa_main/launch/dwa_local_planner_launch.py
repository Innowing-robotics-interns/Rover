from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='path/to/your/config/dwa_local_planner_params.yaml',
        description='Full path to the ROS2 parameters file to use for the node')

    dwa_local_planner_node = Node(
        package='dwa_local_planner',
        executable='dwa_local_planner',
        name='dwa_local_planner',
        output='screen',
        parameters=[config_file]
    )

    ld = LaunchDescription()

    ld.add_action(declare_config_file_cmd)
    ld.add_action(dwa_local_planner_node)

    return ld