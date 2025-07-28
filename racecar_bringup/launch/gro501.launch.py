from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os
import xacro

def launch_setup(context, *args, **kwargs):
    # Package Directories    
    racecar_description = get_package_share_directory('racecar_description')
    racecar_navigation = get_package_share_directory('racecar_navigation')
    # Parse robot description from xacro
    robot_description_file = os.path.join(racecar_description, 'urdf', 'racecar.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    arduinoBridge = Node(package='pb2roscpp',
                         executable='pb2roscpp',
                         name='arduino',
                         output='screen')

    return [arduinoBridge]

    
def generate_launch_description():
    # Declare launch arguments
    
    # Define launch description
    ld = LaunchDescription([

        OpaqueFunction(function=launch_setup),
        
        Node(
            package='racecar_teleop',
            executable='slash_teleop',
            name='teleop',
            output='screen',
        ),

        Node(
            package='racecar_autopilot',
            executable='slash_controller',
            name='controller',
            output='screen',
        ),

        TimerAction(
            period=5.0,  # Adjust the delay duration as needed (in seconds)
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy',
                    parameters=[{'deadzone': 0.05}],
                    arguments=['dev', '/dev/input/js0'],
                    output='screen',
                )
            ]
        )
    ])
    
    return ld
