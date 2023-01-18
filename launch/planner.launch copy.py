import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('planner'), 'config', 'nav2_params.yaml'),
        param_rewrites={},
        convert_types=True
    )

    navigation_bt_params = RewrittenYaml(
        source_file=os.path.join(get_package_share_directory('planner'), 'config', 'navigation_params.yaml'),
        param_rewrites={
            'xml_filepath': os.path.join(get_package_share_directory('planner'), 'config', 'navigation_bt.xml'),
        },
        convert_types=True
    )

    param_file_name1 = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('shelfino_navigation'),
            'config',
            'shelfino1.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    sim = LaunchConfiguration('sim', default='true')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('shelfino_navigation'),
            'map',
            'turtle.yaml'))

    return LaunchDescription([
        # Send obstacles
        Node(
            package='send_obstacles',
            executable='send_obstacles',
            name='send_obstacles'
        ),
        # Inflate obstacles
        # Node(
        #     package='obstacles',
        #     executable='inflate_obstacles_node',
        #     name='inflate_obstacles'
        # ),
        # Planner
        Node(
            package='planner',
            executable='planner',
            name='planner'
        ),
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': sim,
                'namespace': 'shelfino1',
                'use_namespace': 'True',
                'use_composition': 'False',
                'autostart': 'False',
                'params_file': param_file_name1}.items()
        ),
        # NavigationBT
        Node(
            package='planner',
            executable='navigation_bt',
            name='navigation_bt',
            parameters=[navigation_bt_params]
        )
    ])
