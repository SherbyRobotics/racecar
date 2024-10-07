from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directories
    rosbridge_server_pkg_share_dir = get_package_share_directory('rosbridge_server')
    web_video_server_pkg_share_dir = get_package_share_directory('web_video_server')
    racecar_web_interface_pkg_share_dir = get_package_share_directory('racecar_web_interface')
    racecar_bringup_pkg_share_dir = get_package_share_directory('racecar_bringup')
    
    port_argument = DeclareLaunchArgument(
        'port', default_value='9090',
        description='Port for rosbridge websocket'
    )
    

    # Launch rosbridge_websocket_launch.xml from rosbridge_server package
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([racecar_bringup_pkg_share_dir, '/launch/bringup.launch.py'])
    )

    # Launch rosbridge_websocket_launch.xml from rosbridge_server package
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([rosbridge_server_pkg_share_dir, '/launch/rosbridge_websocket_launch.xml']),
        launch_arguments={'port': '9090'}.items()
    )

    #Launch web_video_server node from web_video_server package
    web_video_server_node = Node(
        package='web_video_server',
       executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'address': '10.42.0.1'}]  # Set the address parameter to "10.42.0.1"
    )

    return LaunchDescription([
        bringup_launch,
    	port_argument,
        rosbridge_launch,
        web_video_server_node,
    ])
