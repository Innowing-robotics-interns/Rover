from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    teb_params_file = DeclareLaunchArgument('teb_params_file', default_value='$(find teb_local_planner)/params/teb_params.yaml', description='Path to teb_params.yaml file')

    teb_local_planner_node = Node(
        package='teb_local_planner',
        executable='TebLocalPlannerROS',
        name='teb_local_planner_node',
        output='screen',
        parameters=[{'base_local_planner': 'teb_local_planner/TebLocalPlannerROS', 'controller_frequency': 10.0}]
    )

    return LaunchDescription([
        teb_params_file,
        teb_local_planner_node
    ])