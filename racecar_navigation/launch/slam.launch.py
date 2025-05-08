import os
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    database_path = LaunchConfiguration('database_path').perform(context)
    odom_correction = LaunchConfiguration('odom_correction').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    delete_db = LaunchConfiguration('delete_db').perform(context)

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        arguments= [delete_db],
        parameters=[
            {'database_path': database_path},
            {'frame_id': PathJoinSubstitution([prefix, 'base_footprint'])},
            {'map_frame_id': PathJoinSubstitution([prefix, 'map'])},
            {'odom_frame_id': PathJoinSubstitution([prefix, 'odom','filtered'])},
            {'subscribe_depth': False},
            {'subscribe_rgb': False},
            {'subscribe_scan': True},
            {'use_action_for_goal': True},
            {'queue_size': 10},
            {'topic_queue_size': 1},
            {'approx_sync': True},
            {'RGBD/NeighborLinkRefining': 'true'},
            {'RGBD/ProximityBySpace': 'true'},
            {'RGBD/AngularUpdate': '0.01'},
            {'RGBD/LinearUpdate': '0.01'},
            {'Grid/FromDepth': 'false'},
            {'Grid/CellSize': '0.1'},
            {'Reg/Force3DoF': 'true'},
            {'Reg/Strategy': '1'},
            {'DbSqlite3/InMemory': 'true'},
            {'RGBD/OptimizeFromGraphEnd': 'false'},
            {'RGBD/ProximityMaxGraphDepth': '0'},
            {'RGBD/SavedLocalizationIgnored': 'true'},
            {'Icp/VoxelSize': '0.05'},
            {'Icp/MaxCorrespondenceDistance': '0.1'},
            {'use_sim_time': use_sim_time},
            {'Mem/IncrementalMemory': 'false' if localization == 'true' else 'true'},
        ],
        remappings=[
            ('scan', f'{prefix}/scan'),
            ('grid_map', 'map'),
            ('move_base', 'move_base')
        ]
    )

    return [rtabmap_node]


def generate_launch_description():


    prefix_arg = DeclareLaunchArgument('prefix', default_value='racecar')
    localization_arg = DeclareLaunchArgument('localization', default_value='false')
    database_path_arg = DeclareLaunchArgument('database_path', default_value='~/.ros/validation.db')
    odom_correction_arg = DeclareLaunchArgument('odom_correction', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    delete_db_arg = DeclareLaunchArgument('delete_db', default_value='--uerror')

    return LaunchDescription([
        prefix_arg,
        localization_arg,
        database_path_arg,
        odom_correction_arg,
        use_sim_time_arg,
        delete_db_arg,
        OpaqueFunction(function=launch_setup)
    ])