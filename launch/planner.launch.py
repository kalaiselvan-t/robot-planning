from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='send_obstacles',
            executable='send_obstacles',
            name='send_obstacles'
        ),
        Node(
            package='obstacles',
            executable='inflate_obstacles_node',
            name='inflate_obstacles'
        ),
        Node(
            package='planner',
            executable='planner',
            name='planner'
        ),
    ])
