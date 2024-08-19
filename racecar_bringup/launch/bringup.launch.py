from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():


    # Package Directories    
    racecar_description = get_package_share_directory('racecar_description')
    # Parse robot description from xacro
    robot_description_file = os.path.join(racecar_description, 'urdf', 'racecar.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}


    return LaunchDescription([

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pb2ros2',
                    executable='arduino_agent',
                    name='arduino',
                    output='screen',
                ),
            ],
        ),


        Node(
            package='racecar_bringup',
            executable='arduino_sensors',
            name='arduino_sensors',
            output='screen',
            remappings=[('/raw_odom', 'prop_sensors')],
        ),

        Node(
            name='lidar',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node(
            package='racecar_web_interface',
            executable='ros2_publisher_camera',
            name='camera',
        ),
        
    ])
