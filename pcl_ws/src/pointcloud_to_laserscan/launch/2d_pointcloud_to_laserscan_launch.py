from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scan', default_value='scan',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in','/2d_pointcloud'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
            }],
            name='pointcloud_to_laserscan'
        )
    ])
