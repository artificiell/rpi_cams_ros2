#!/usr/bin/env python3
"""
The "bringup" everything launch file: starts the camera ONCE, then composes
whichever feature nodes are needed on top of it (image2scan, ArUco detection).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    frame_rate = LaunchConfiguration('frame_rate')
    flip_image = LaunchConfiguration('flip_image')
    display = LaunchConfiguration('display')
    server_port = LaunchConfiguration('server_port')
    camera_height = LaunchConfiguration('camera_height')
    camera_offset = LaunchConfiguration('camera_offset')
    use_image2scan = LaunchConfiguration('use_image2scan')
    use_aruco = LaunchConfiguration('use_aruco')
    marker_size = LaunchConfiguration('marker_size')
    aruco_dictionary_id = LaunchConfiguration('aruco_dictionary_id')

    declared_args = [
        DeclareLaunchArgument('robot_ns', default_value = 'rpi'),
        DeclareLaunchArgument('width', default_value = '640'),
        DeclareLaunchArgument('height', default_value = '480'),
        DeclareLaunchArgument('frame_rate', default_value = '15.0'),
        DeclareLaunchArgument('flip_image', default_value = 'true'),
        DeclareLaunchArgument('display', default_value = 'true'),
        DeclareLaunchArgument('server_port', default_value = '9090'),
        DeclareLaunchArgument('camera_height', default_value = '0.1'),
        DeclareLaunchArgument('camera_offset', default_value = '0.04'),
        DeclareLaunchArgument('marker_size', default_value = '0.1'),
        DeclareLaunchArgument('aruco_dictionary_id', default_value = 'DICT_ARUCO_ORIGINAL'),
    ]

    # Include camera launch (once)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rpi_cam'), 'launch', 'camera.launch.py'])
        ),
        launch_arguments={
            'robot_ns': robot_ns,
            'width': width,
            'height': height,
            'frame_rate': frame_rate,
            'flip_image': flip_image,
            'display': display,
            'server_port': server_port,
        }.items()
    )

    # Include image2scan launch (no camera launch, as it is already include from above)
    image2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rpi_cam'), 'launch', 'image2scan.launch.py'])
        ),
        launch_arguments={
            'robot_ns': robot_ns,
            'camera_height': camera_height,
            'camera_offset': camera_offset,
            'launch_camera': 'false',
        }.items(),
    )

    # Include detection launch (no camera launch)
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('aruco_marker'), 'launch', 'detection.launch.py'])
        ),
        launch_arguments={
            'robot_ns': robot_ns,
            'marker_size': marker_size,
            'aruco_dictionary_id': aruco_dictionary_id,
            'display': display,
            'launch_camera': 'false',
        }.items(),
    )

    return LaunchDescription(declared_args + [
        camera_launch,
        image2scan_launch,
        aruco_launch,
    ])
