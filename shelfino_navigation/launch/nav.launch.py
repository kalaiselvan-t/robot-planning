# Copyright 2019 Open Source Robotics Foundation, Inc.
# Author: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_yaml = os.path.join(get_package_share_directory('shelfino_navigation'), 'config','controller.yaml')

    rviz_config_dir = os.path.join(
    get_package_share_directory('nav2_bringup'),
    'rviz',
    'nav2_default_view.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='nav2_map_server',
            namespace='shelfino2',
            parameters=[{'yaml_filename': "/home/placido/ros2_ws/install/shelfino_navigation/share/shelfino_navigation/map/ufficio.yaml"}],
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')],
            output='screen'),
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='nav2_planner',
        #     namespace='gazebo',
        #     parameters=[controller_yaml],
        #     output='screen'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='nav2_controller',
            namespace='shelfino2',
            parameters=[controller_yaml],
            output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            namespace='shelfino2',
            parameters=[{'autostart': True},
                        {'node_names': ['nav2_map_server','nav2_controller']}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': False}],
            output='screen')
    ])
