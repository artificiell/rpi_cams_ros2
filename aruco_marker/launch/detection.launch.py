#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    frame_rate = LaunchConfiguration('frame_rate')
    flip_image = LaunchConfiguration('flip_image')
    display = LaunchConfiguration('display')
    server_port = LaunchConfiguration('server_port')
    launch_camera = LaunchConfiguration('launch_camera')
    marker_size = LaunchConfiguration('marker_size')
    aruco_dictionary_id = LaunchConfiguration('aruco_dictionary_id')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'rpi'
    )
    width_arg = DeclareLaunchArgument(
        'width',
        default_value = '640'
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value = '480'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value = '15.0'
    )
    flip_image_arg = DeclareLaunchArgument(
        'flip_image',
        default_value = 'true'
    )
    display_arg = DeclareLaunchArgument(
        'display',
        default_value = 'true',
        description = 'Whether web_video_server should also be launched (and annotated ArUco amerkers being displayed).'
    )
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value = '9090'
    )
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value = 'true',
        description = 'Whether this file should also launch up the camera.'
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value = '0.1',
        description = 'Size of the ArUco markers in meters.'
    )
    aruco_dictionary_id_arg = DeclareLaunchArgument(
        'aruco_dictionary_id',
        default_value = 'DICT_ARUCO_ORIGINAL',
        description = 'OpenCV ArUco dictionary used to generate the markers.'
    )

    # Include camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rpi_cam'),
                'launch',
                'camera.launch.py'
            ])
        ),
        launch_arguments = {
            'robot_ns': robot_ns,
            'width': width,
            'height': height,
            'frame_rate': frame_rate,
            'flip_image': flip_image,
            'display': display,
            'server_port': server_port,
        }.items(),
        condition = IfCondition(launch_camera)
    )

    # ArUco detection node
    detection_node = Node(
        package='aruco_marker',
        namespace=robot_ns,
        executable='detect',
        name='aruco_detection_node',
        parameters = [{
            'marker_size': marker_size,
            'aruco_dictionary_id': aruco_dictionary_id,
            'display': display,
        }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        width_arg,
        height_arg,
        frame_rate_arg,
        flip_image_arg,
        display_arg,
        server_port_arg,
        launch_camera_arg,
        marker_size_arg,
        aruco_dictionary_id_arg,
        camera_launch,
        detection_node
    ])
