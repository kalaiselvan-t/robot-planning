#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    shelfino_cartographer_prefix = get_package_share_directory(
        'shelfino_node')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(shelfino_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='shelfino.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory(
        'shelfino_node'), 'config', 'shelfino_slam.rviz')

    sim = LaunchConfiguration('sim')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_gazebo'), 'models')
    rviz_model = os.path.join(gazebo_models_path, 'shelfino', 'model.urdf')
    robot_description = ParameterValue(rviz_model, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        
        DeclareLaunchArgument(name='sim', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),

        Node(
	        package='shelfino_node',
	        executable='shelfino_node',
            name='shelfino_node',
            condition=UnlessCondition(sim)
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': sim}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': sim}],
            arguments=['-resolution', resolution,
                       '-publish_period_sec', publish_period_sec]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': sim}],
            output='screen'
        ),
    ])