from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='cartographer_config').find('cartographer_config')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'robot_2d.lua'
    pbstream_file = os.path.join(pkg_share, 'pbstream')+"/output2.pbstream"

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', pbstream_file,
            ],
            remappings=[
                ('scan', 'scan'),
            ],
        ),
    ])
