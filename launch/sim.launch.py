#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')
    namespace = LaunchConfiguration('namespace', default='')

    world_file_name = 'hexagon.world'
    world = os.path.join(get_package_share_directory('shelfino_gazebo'),
                         'worlds', world_file_name)

    params_shelfino1 = LaunchConfiguration(
        'params_shelfino1',
        default=os.path.join(
            get_package_share_directory('planner'),
            'config',
            'shelfino1.yaml'))

    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # rviz_config1 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino1.rviz')
    # rviz_config2 = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino2.rviz')

    rviz_config_shelfino1 = os.path.join(
        get_package_share_directory('planner'),
        'rviz',
        'shelfino1_nav.rviz')

    # rviz_config_shelfino2 = os.path.join(
    #     get_package_share_directory('shelfino_navigation'),
    #     'rviz',
    #     'shelfino2_nav.rviz')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
                    # ('/goal_pose', 'goal_pose'),
                    # ('/clicked_point', 'clicked_point'),
                    # ('/initialpose', 'initialpose')]

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('shelfino_navigation'),
            'map',
            'turtle.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
            description='Flag to enable gazebo visualization'),

        DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
            description='Flag to enable rviz visualization'),

        DeclareLaunchArgument(name='use_sim_time', default_value='true', choices=['true', 'false'],
            description='Flag to toggle between real robot and simulation'),

        DeclareLaunchArgument(name='remote', default_value='false', choices=['true', 'false'],
            description='Flag to toggle between navigation stack running on robot or locally'),

        DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
            description='Flag to toggle between navigation stack running on robot or locally'),

        DeclareLaunchArgument('model', default_value=[os.path.join(gazebo_models_path, 'shelfino'),'/model.sdf']),
        LogInfo(msg=LaunchConfiguration('model')),

        DeclareLaunchArgument('namespace', default_value=namespace,
            description='Top-level namespace'),

        PushRosNamespace(namespace=namespace),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'namespace': 'shelfino1',
                'use_namespace': 'True',
                'use_composition': 'False',
                'autostart': 'False',
                # 'slam': 'True',
                # 'params_file': os.path.join(get_package_share_directory('planner'), 'config', 'shelfino1.yaml')}.items(),
                'params_file': params_shelfino1}.items(),
            condition=UnlessCondition(remote),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
        #     launch_arguments={
        #         'namespace': 'shelfino1',
        #         'use_sim_time': use_sim_time,
        #         'autostart': 'False',
        #         'params_file': params_shelfino1,
        #         'use_composition': 'False',
        #         'use_respawn': 'False',
        #         'container_name': 'nav2_container'}.items(),
        #         condition=UnlessCondition(remote),
        # ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', LaunchConfiguration('model'),
                       '-entity', 'shelfino1',
                       '-robot_namespace', 'shelfino1',
                       '-x', '0',
                       '-y', '0']
        ),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-file', LaunchConfiguration('model'),
        #                '-entity', 'shelfino2',
        #                '-robot_namespace', 'shelfino2',
        #                '-x', '1',
        #                '-y', '0']
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': 'shelfino1'}.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time,
        #                       'robot_id': 'shelfino2'}.items()
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace='shelfino1',
            arguments=['-d', rviz_config_shelfino1],
            condition=IfCondition(rviz),
            remappings=remappings
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     namespace='shelfino2',
        #     arguments=['-d', rviz_config_shelfino2],
        #     condition=IfCondition(rviz),
        #     remappings=remappings
        # ),

        Node(
            package='send_obstacles',
            executable='send_obstacles'
        ),

        Node(
            package='send_borders',
            executable='send_borders'
        ),

        Node(
            package='send_gates',
            executable='send_gates'
        ),
    ])
