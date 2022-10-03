#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_gazebo'), 'models')
    rviz_model = os.path.join(gazebo_models_path, 'shelfino', 'model.urdf')
    robot_description = ParameterValue(rviz_model, value_type=str)

    return LaunchDescription([
        Node(
            package='shelfino_node',
#            namespace='shelfino2',
            executable='shelfino_node'
        ),
        Node(
            package='rviz2',
#            namespace='shelfino2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("shelfino_node"),
                                   'config', 'shelfino.rviz')]
        )
    ])
