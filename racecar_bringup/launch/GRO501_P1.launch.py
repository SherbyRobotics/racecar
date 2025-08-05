from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
    # Declare launch arguments
    DeclareLaunchArgument('start_robotStatePublisher', default_value='False'),
    DeclareLaunchArgument('start_arduinoBridge', default_value='True'),
    DeclareLaunchArgument('start_arduinoSensor', default_value='False'),
    DeclareLaunchArgument('start_lidar', default_value='False'),
    DeclareLaunchArgument('start_camera', default_value='False'),
    DeclareLaunchArgument('start_magwick', default_value='False'),
    DeclareLaunchArgument('start_kalma', default_value='False'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py']),
            launch_arguments={'start_robotStatePublisher': LaunchConfiguration('start_robotStatePublisher'),
                              'start_arduinoBridge': LaunchConfiguration('start_arduinoBridge'),
                              'start_arduinoSensor': LaunchConfiguration('start_arduinoSensor'),
                              'start_lidar': LaunchConfiguration('start_lidar'),
                              'start_camera': LaunchConfiguration('start_camera'),
                              'start_magwick': LaunchConfiguration('start_magwick'),
                              'start_kalma': LaunchConfiguration('start_kalma')}.items()
        )])
