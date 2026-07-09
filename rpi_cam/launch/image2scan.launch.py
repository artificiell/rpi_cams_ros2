#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


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
    camera_height = LaunchConfiguration('camera_height')
    camera_offset = LaunchConfiguration('camera_offset')
    
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
        default_value = 'false',
        description = 'Whether to also launch web_video_server for browser viewing.'
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
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value = '0.1',
        description = 'Camera height above the floor, in meters.'
    )
    camera_offset_arg = DeclareLaunchArgument(
        'camera_offset',
        default_value = '0.04',
        description = "How far forward of the robot's reference frame "
                      "(base_link) the camera is mounted, in meters."
    )

    # Include camera lanch
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

    # Image to laser scan node
    image2scan_node = Node(
        package = 'rpi_cam',
        namespace = robot_ns,
        executable = 'image2scan',
        name = 'rpi_image_to_laserscan_node',
        parameters = [{
            'camera_height': camera_height,
            'camera_offset': camera_offset,
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
        camera_height_arg,
        camera_offset_arg,
        camera_launch,
        image2scan_node
    ])
