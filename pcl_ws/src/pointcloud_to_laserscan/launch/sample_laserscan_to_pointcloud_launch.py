from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scan', default_value='scan',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', '/scans'),
                        ('cloud', '/pointcloud')],
            parameters=[{'target_frame': 'livox_frame', 'transform_tolerance': 0.01}]
        ),
    ])
