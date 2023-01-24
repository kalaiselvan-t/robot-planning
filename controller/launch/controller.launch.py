import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='false')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('shelfino_gazebo'), 'launch'), '/multi_shelfino.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'gui': gui,
                'rviz': rviz,
            }.items()
        ),
        Node(
            package='obstacles',
            executable='inflate_obstacles',
            name='inflate_obstacles'
        ),
        # Node(
        #     package='controller',
        #     executable='controller',
        #     name='controller',
        #     output='screen'
        # ),
    ])
