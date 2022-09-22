import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("ros2_interface"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory("ros2_interface"),
                                   'config', 'shelfino_slam.rviz'),
        description='Full path to the ROS2 rviz config file')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    start_shelfino = Node(
	package='ros2_interface',
	executable='shelfino_node')
	
    start_rviz = Node(
	package='rviz2',
	executable='rviz2',
	arguments=['-d', rviz_config_file])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_shelfino)
    ld.add_action(start_rviz)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
