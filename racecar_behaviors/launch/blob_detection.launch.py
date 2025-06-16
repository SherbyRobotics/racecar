from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    #Declare launch arguments
    prefix = LaunchConfiguration('prefix').perform(context)
    debug = LaunchConfiguration('debug').perform(context)

     # Define nodes for behaviors
    republish_raspicam_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_raspicam',
        remappings=[('in', 'racecar/camera'),
                   ('out', 'raspicam_node/image')],
        arguments=['raw']
    )

    laserscan_to_pointcloud_node = Node(
        package='racecar_behaviors',
        executable='laserscan_to_pointcloud',
        name='laserscan_to_pointcloud',
        remappings=[('/scan', '/racecar/scan'), ('converted_pc', 'scan_cloud')],
        arguments=["--ros-args", "--log-level", 'laserscan_to_pointcloud:=warn']
    )

    pointcloud_to_depthimage_node = Node(
        package='rtabmap_util',
        executable='pointcloud_to_depthimage',
        name='pointcloud_to_depthimage',
        parameters=[{'fixed_frame_id': 'racecar/odom/filtered', 'fill_holes_size': 4, 'topic_queue_size': 10}],
        remappings=[('camera_info', 'racecar/camera_info'), ('cloud', 'scan_cloud'),
                    ('image', 'raspicam_node/depth_registered'), ('image_raw', 'raspicam_node/depth_registered_raw')]
    )

    blob_detector_node = Node(
        package='racecar_behaviors',
        executable='blob_detector',
        name='blob_detector',
        output='screen',
        remappings=[('image', 'raspicam_node/image'), ('camera_info', 'racecar/camera_info'),
                    ('depth', 'raspicam_node/depth_registered')],
        parameters=[{'map_frame_id': 'racecar/odom', 'frame_id': 'racecar/base_footprint',
                     'object_frame_id': 'racecar/object', 'color_hue': 160}]
    )

    # Define debug nodes
    point_cloud_xyzrgb_node = Node(
        package='nodelet',
        executable='nodelet',
        name='point_cloud_xyzrgb',
        arguments=['standalone', 'rtabmap_util/point_cloud_xyzrgb'],
        parameters=[{'approx_sync': False}],
        remappings=[('cloud', 'raspicam_node/registered_cloud'), ('depth/image', 'raspicam_node/depth_registered_raw'),
                    ('rgb/image', 'raspicam_node/image'), ('rgb/camera_info', 'raspicam_node/camera_info')],
        condition=IfCondition(debug)
    )

    image_view_detections_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view_detections',
        remappings=[('image', '/racecar/image_detections')],
        condition=IfCondition(debug)
    )

    image_view_depth_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view_depth',
        remappings=[('image', 'raspicam_node/depth_registered')],
        condition=IfCondition(debug)
    )

    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        condition=IfCondition(debug)
    )

    rviz_node = Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        arguments=['-d', 'racecar_bringup/config/bringup.rviz'],
        condition=IfCondition(debug)
    )

    # Group debug nodes
    debug_group = GroupAction([
        point_cloud_xyzrgb_node,
        image_view_detections_node,
        image_view_depth_node,
        rqt_reconfigure_node,
        rviz_node
    ])

    return [republish_raspicam_node,
            laserscan_to_pointcloud_node,
            pointcloud_to_depthimage_node,
            blob_detector_node,
            debug_group]


def generate_launch_description():
    # Declare launch arguments
    prefix_arg = DeclareLaunchArgument('prefix', default_value='racecar')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')

    # Define launch description
    ld = LaunchDescription([
        prefix_arg,
        debug_arg,
        OpaqueFunction(function=launch_setup)
    ])

    return ld

