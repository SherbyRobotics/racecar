import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

def launch_setup(context, *args, **kwargs):
    # Declare launch arguments
    prefix = LaunchConfiguration('prefix').perform(context)

    # Package Directories    
    racecar_description = get_package_share_directory('racecar_description')
    racecar_gazebo = get_package_share_directory('racecar_gazebo')

    # Parse robot description from xacro
    robot_description_file = os.path.join(racecar_description, 'urdf', 'racecar.xacro')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_config2 = xacro.process_file(robot_description_file, mappings={'robot_name' : "voiture_rapide", 'prefix': "voiture_rapide", 'color': 'Red'})
    robot_description2 = {'robot_description': robot_description_config2.toxml()}

    # Ros2 bridge
    bridge_config = os.path.join(racecar_description, 'config', 'ros_bridge.yaml')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
        # namespace=prefix
    )

    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher2',
        output='both',
        parameters=[robot_description2],
        remappings=[('robot_description', 'robot_description2')],
        # namespace=prefix
    )

    # Bridge 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
        namespace=prefix
    )

    # Spawn
    spawn1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'racecar',
            '-topic', 'robot_description',
            '-x', '1.5',
            '-y', '0.75',
        ],
        output='screen',
        # namespace=prefix
    )

    spawn2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'racecar2',
            '-topic', 'robot_description2',
            '-x', '1.5',
            '-y', '-0.25',
        ],
        output='screen',
        # namespace=prefix
    )

    cmd_vel_arb = Node(
        package='racecar_bringup',
        executable='cmd_vel_arbitration',
        remappings=[(f'/{prefix}/cmd_vel_output', f'/{prefix}/cmd_vel')],
        namespace=prefix
    )

    cmd_vel_arb2 = Node(
        package='racecar_bringup',
        executable='cmd_vel_arbitration',
        remappings=[(f'/{prefix}2/cmd_vel_output', f'/{prefix}2/cmd_vel')],
        namespace=prefix+'2'
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


    gaz_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(racecar_gazebo, 'launch', 'gazebo_control.launch.py')]),
    )

    return [
        robot_state_publisher,
        robot_state_publisher2,
        bridge,
        spawn1,
        spawn2,
        cmd_vel_arb,
        cmd_vel_arb2,
        joystick,
        teleop,
        gaz_control
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='racecar'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()