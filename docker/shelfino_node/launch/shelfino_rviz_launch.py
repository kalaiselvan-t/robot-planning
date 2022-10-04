#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_gazebo'), 'launch')

    return LaunchDescription([
        Node(
            package='shelfino_node',
#            namespace='shelfino2',
            executable='shelfino_node'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='rviz2',
#            namespace='shelfino2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("shelfino_node"),
                                   'rviz', 'shelfino.rviz')]
        )
    ])
