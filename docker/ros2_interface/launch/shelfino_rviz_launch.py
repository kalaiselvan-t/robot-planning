import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_interface',
            namespace='shelfino2',
            executable='shelfino_node'
        ),
        Node(
            package='rviz2',
            namespace='shelfino2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("ros2_interface"),
                                   'config', 'shelfino.rviz')]
        )
    ])
