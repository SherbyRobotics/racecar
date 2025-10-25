import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xml.dom.minidom import Document


def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    prefix = LaunchConfiguration('prefix').perform(context)

    # Package Directories
    racecar_description = get_package_share_directory('racecar_description')
    # racecar_gazebo = get_package_share_directory('racecar_gazebo')  # NOTE: Unused.
    racecar_navigation = get_package_share_directory('racecar_navigation')

    # Parse robot description from xacro
    robot_description_file = os.path.join(racecar_description, 'urdf', 'racecar.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    assert isinstance(robot_description_config, Document)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Ros2 bridge
    bridge_config = os.path.join(racecar_description, 'config', 'ros_bridge.yaml')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Bridge 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
        namespace=prefix
    )
    
    # Bridge image 
    imageBridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/racecar/camera'],
        output='screen'
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'robot',
            '-topic', 'robot_description',
        ],
        output='screen',
    )

    cmd_vel_arb = Node(
        package='racecar_bringup',
        executable='cmd_vel_arbitration',
        remappings=[(f'/{prefix}/cmd_vel_output', f'/{prefix}/cmd_vel')],
        namespace=prefix
    )

    joystick = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.2},
                    {'autorepeat_rate': 0.0},
                    {'coalesce_interval': 0.01}],
        namespace=prefix
    )


    teleop = Node(
            package='racecar_teleop',
            executable='slash_teleop',
            name='racecar_teleop',
            remappings=[(f'/{prefix}/ctl_ref', f'/{prefix}/cmd_vel_abtr_0')],
            namespace=prefix   
    )


    # gaz_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(racecar_gazebo, 'launch', 'gazebo_control.launch.py')]),
    # )  # NOTE: Does nothing.

    kalmanFilter = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(racecar_navigation, 'launch', 'kalmanFilter.launch.py')]),
                        launch_arguments={"odom_topic":f'/{prefix}/odom/filtered',
                                          "use_sim_time":"True"}.items()
                    )

    return [
        robot_state_publisher,
        bridge,
        imageBridge,
        spawn,
        cmd_vel_arb,
        joystick,
        teleop,
        # gaz_control,
        kalmanFilter
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='racecar'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
