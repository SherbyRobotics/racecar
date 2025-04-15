import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    use_sim_time= LaunchConfiguration('use_sim_time').perform(context)

    # Set nav2 params file
    nav2_params_file = os.path.join(get_package_share_directory('racecar_navigation'), 'resource','nav2_params.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('racecar_navigation'), 'launch', 'navigation_stack.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items())
    
    nav2_group = GroupAction([
        SetRemap(src='/cmd_vel_nav', dst=f'/{prefix}/cmd_vel_nav'),
        SetRemap(src='/cmd_vel_teleop', dst=f'/{prefix}/cmd_vel_teleop'),
        SetRemap(src='/cmd_vel', dst=f'/{prefix}/{cmd_vel_topic}'),
        nav2_bringup,
    ])

    return [nav2_group]

def generate_launch_description():
    # Declare launch arguments
    prefix_arg = DeclareLaunchArgument('prefix', default_value='racecar')
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel_abtr_5')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='odom')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Define launch description
    ld = LaunchDescription([
        use_sim_time_arg,
        prefix_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])

    return ld