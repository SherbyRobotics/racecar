import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    prefix = LaunchConfiguration('prefix', default='racecar')
    cmd_vel_topic_arg = LaunchConfiguration('cmd_vel_topic', default='cmd_vel_abtr_5')
    odom_topic_arg = LaunchConfiguration('odom_topic', default='odom')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Set nav2 params file
    nav2_params_file = os.path.join(get_package_share_directory('racecar_navigation'), 'nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items())
    # Define launch description
    ld = LaunchDescription([
        use_sim_time_arg,
        nav2_bringup
    ])

    return ld

