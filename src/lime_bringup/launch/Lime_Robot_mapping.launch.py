from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml


def generate_launch_description():
    mode = LaunchConfiguration('mode', default='navigation')
    cartographer_dir = get_package_share_directory('cartographer_config')

    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', ['/scan']),
                        ('cloud', ['/cloud'])],
            parameters=[{'target_frame': 'laser_link', 'transform_tolerance': 0.01}]
        ),
        Node(
            package='data_transmission',
            executable='serial_node',
        ),
        Node(
            package='model_display',
            executable='model_display',
        ),
        # Node(
        #     package='slam',
        #     executable='slam_node',
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_dir,'/launch','/cartographer.launch.py'])
        )
    ])
