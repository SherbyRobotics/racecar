import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    
    pkg_share = get_package_share_directory('racecar_navigation')
    robot_localization_file_path = os.path.join(pkg_share,'resource','dual_ekf_params.yaml')

    with open(robot_localization_file_path, 'r') as f:
        fusion_params = yaml.safe_load(f)['sensor_fusion']['ros_parameters']

    fusion_params["use_sim_time"] = LaunchConfiguration('use_sim_time')
    
    
    kalmaFilter = Node(package='robot_localization',
                       executable='ekf_node',
                       name='sensor_fusion',
                       output='screen',
                       emulate_tty=True,
                       parameters=[fusion_params],
                       remappings=[('/odometry/filtered', LaunchConfiguration('odom_topic'))]
                    )
    
    return [kalmaFilter]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odom/filtered')

    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        OpaqueFunction(function=launch_setup)
    ])