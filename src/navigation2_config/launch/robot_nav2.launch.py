import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    navigation2_config_dir = get_package_share_directory('navigation2_config')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')


    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(navigation2_config_dir,'maps','map2.yaml'))

    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(navigation2_config_dir,'param','nav2_param2.yaml'))

    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                }.items(),
        )
    

    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    autolocalization_node = Node(
            package='navigation2_config',  
            executable='autolocalization_node', 
            name='autolocalization_node',
            output='screen')
    
    cmd_nav_node = Node(
            package='navigation2_config',
            executable='cmd_nav_node',
        )

    
    return LaunchDescription([nav2_bringup_launch,rviz_node,autolocalization_node])