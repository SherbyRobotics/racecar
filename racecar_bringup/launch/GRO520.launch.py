import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    racecar_bringup = get_package_share_directory('racecar_bringup')
    
    return LaunchDescription([

    # Declare launch arguments
    DeclareLaunchArgument('start_robotStatePublisher', default_value='False'),
    DeclareLaunchArgument('start_arduinoBridge', default_value='True'),
    DeclareLaunchArgument('start_arduinoSensor', default_value='True'),
    DeclareLaunchArgument('start_lidar', default_value='False'),
    DeclareLaunchArgument('start_camera', default_value='False'),
    DeclareLaunchArgument('start_magwick', default_value='False'),
    DeclareLaunchArgument('start_kalma', default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch', 'bringup.launch.py')]),
            launch_arguments={'start_robotStatePublisher': LaunchConfiguration('start_robotStatePublisher'),
                              'start_arduinoBridge': LaunchConfiguration('start_arduinoBridge'),
                              'start_arduinoSensor': LaunchConfiguration('start_arduinoSensor'),
                              'start_lidar': LaunchConfiguration('start_lidar'),
                              'start_camera': LaunchConfiguration('start_camera'),
                              'start_magwick': LaunchConfiguration('start_magwick'),
                              'start_kalma': LaunchConfiguration('start_kalma')}.items()
        )])
