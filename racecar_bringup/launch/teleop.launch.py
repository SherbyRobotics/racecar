import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    racecar_bringup = get_package_share_directory('racecar_bringup')

    return LaunchDescription([
    # Declare launch arguments
    DeclareLaunchArgument('start_robotStatePublisher', default_value='True'),
    DeclareLaunchArgument('start_arduinoBridge', default_value='True'),
    DeclareLaunchArgument('start_arduinoSensor', default_value='True'),
    DeclareLaunchArgument('start_lidar', default_value='True'),
    DeclareLaunchArgument('start_camera', default_value='True'),
    DeclareLaunchArgument('start_magwick', default_value='True'),
    DeclareLaunchArgument('start_kalma', default_value='True'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch', 'bringup.launch.py')]),
            launch_arguments={'start_robotStatePublisher': LaunchConfiguration('start_robotStatePublisher'),
                              'start_arduinoBridge': LaunchConfiguration('start_arduinoBridge'),
                              'start_arduinoSensor': LaunchConfiguration('start_arduinoSensor'),
                              'start_lidar': LaunchConfiguration('start_lidar'),
                              'start_camera': LaunchConfiguration('start_camera'),
                              'start_magwick': LaunchConfiguration('start_magwick'),
                              'start_kalma': LaunchConfiguration('start_kalma')}.items()
        ),
        
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
                    executable='game_controller_node',
                    name='joy',
                    parameters=[{'deadzone': 0.05}],
                    arguments=['dev', '/dev/input/js0'],
                    output='screen',
                ),
            ]
        )
    ])
