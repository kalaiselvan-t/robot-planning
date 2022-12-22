#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world_file_name = 'turtle.world'
    world = os.path.join(get_package_share_directory('shelfino_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    rviz_config = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino.rviz')

    robot_id = LaunchConfiguration('robot_id', default='gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                description='Flag to enable gazebo visualization'),

        DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
                                description='Flag to enable rviz visualization'),

        DeclareLaunchArgument(name='robot_id', default_value='gazebo', 
                                description='Robot name'),

        DeclareLaunchArgument('model', default_value=[os.path.join(gazebo_models_path, 'shelfino'),'/',LaunchConfiguration('robot_id'), '.sdf']),
        LogInfo(msg=LaunchConfiguration('model')),

        ExecuteProcess(
            cmd=[[
                'xacro ',
                 os.path.join(get_package_share_directory('shelfino_description'),'models','shelfino','model.sdf.xacro'),
                ' robot_name:=',
                LaunchConfiguration('robot_id'),
                ' > ',
                LaunchConfiguration('model')
            ]],
            shell=True
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui)
        ),
    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', robot_id]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_id}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz)
        )
    ])
