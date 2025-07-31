from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
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

    # Robot state publisher
    robotStatePublisher = Node(package='robot_state_publisher',
                               executable='robot_state_publisher',
                               name='robot_state_publisher',
                               output='both',
                               parameters=[robot_description],
                               condition=IfCondition(LaunchConfiguration('start_robotStatePublisher'))
                               )
    
    arduinoBridge = Node(package='pb2roscpp',
                         executable='pb2roscpp',
                         name='arduino',
                         output='screen',
                         condition=IfCondition(LaunchConfiguration('start_arduinoBridge'))
                         )
    
    
    arduinoSensor = Node(package='racecar_bringup',
                         executable='arduino_sensors',
                         name='arduino_sensors',
                         output='screen',
                         remappings=[('/raw_odom', 'prop_sensors'),
                                     ('/odom', '/racecar/odom')],
                         condition=IfCondition(LaunchConfiguration('start_arduinoSensor'))
                         )
    
    
    lidar = Node(name='lidar',
                 package='rplidar_ros',
                 executable='rplidar_composition',
                 output='screen',
                 parameters=[{'serial_port': '/dev/ttyUSB0',
                              'serial_baudrate': 115200,
                              'frame_id': 'racecar/base_laser',
                              'inverted': False,
                              'angle_compensate': True}],
                 remappings=[('/scan', '/racecar/scan')],
                 condition=IfCondition(LaunchConfiguration('start_lidar'))
                 )
    
    camera =   Node(package='v4l2_camera',
                    executable='v4l2_camera_node',
                    name='camera',
                    parameters=[{'camera_frame_id' : 'racecar/camera_optical_link',
                                 'saturation' : 100,}],
                    remappings=[('image_raw', 'racecar/camera'),
                                ('camera_info', 'racecar/camera_info')],
                    condition=IfCondition(LaunchConfiguration('start_camera'))
                    )
    
    magwick = Node(package='imu_filter_madgwick',
                   executable='imu_filter_madgwick_node',
                   name='imu_filter_node',
                   output='screen',
                   parameters=[{'use_mag':True},
                               {'world_frame':'enu'},
                               {'publish_tf':False}],
                   remappings=[("/imu/data","racecar/imu")],
                   condition=IfCondition(LaunchConfiguration('start_magwick'))
                   )
    
    kalmaFilter =  IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(racecar_navigation, 'launch', 'kalmanFilter.launch.py')]),
    launch_arguments={"odom_topic":'/racecar/odom/filtered', "use_sim_time":"false"}.items(),
    condition=IfCondition(LaunchConfiguration('start_kalma')
                                            ))
                                                             
    return [robotStatePublisher,
            arduinoBridge,
            arduinoSensor,
            lidar,
            camera,
            magwick,
            kalmaFilter]

    
def generate_launch_description():
    # Declare launch arguments
    start_robot_state_publisher_arg = DeclareLaunchArgument('start_robotStatePublisher', default_value='True')
    start_arduino_bridge_arg = DeclareLaunchArgument('start_arduinoBridge', default_value='True')
    start_arduino_sensor_arg = DeclareLaunchArgument('start_arduinoSensor', default_value='True')
    start_lidar_arg = DeclareLaunchArgument('start_lidar', default_value='True')
    start_camera_arg = DeclareLaunchArgument('start_camera', default_value='True')
    start_magwick_arg = DeclareLaunchArgument('start_magwick', default_value='True')
    start_kalma_arg = DeclareLaunchArgument('start_kalma', default_value='True')
    
    # Define launch description
    ld = LaunchDescription([
        start_robot_state_publisher_arg,
        start_arduino_bridge_arg,
        start_arduino_sensor_arg,
        start_lidar_arg,
        start_camera_arg,
        start_magwick_arg,
        start_kalma_arg,
        OpaqueFunction(function=launch_setup)
    ])
    
    return ld
