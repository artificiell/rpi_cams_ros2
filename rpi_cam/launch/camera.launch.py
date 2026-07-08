#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        default_value = 'True'
    )
    display_arg = DeclareLaunchArgument(
        'display',
        default_value = 'True',
        description = 'Whether to also launch web_video_server for browser viewing.'
    )
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value = '9090'
    )
    # Camera node
    camera_node = Node(
        package = 'rpi_cam',
        namespace = robot_ns,
        executable = 'camera',
        name = 'rpi_camera_node',
        parameters = [{
            'width': width,
            'height': height,
            'frame_rate': frame_rate,
            'flip': flip_image,
        }]
    )
    # Web video server node (stock ROS 2 package)
    web_video_server_node = Node(
        package = 'web_video_server',
        namespace = robot_ns,
        executable = 'web_video_server',
        name = 'web_video_server',
        parameters = [{
            'port': server_port,
        }],
        condition = IfCondition(display)
    )
    return LaunchDescription([
        robot_ns_launch_arg,
        width_arg,
        height_arg,
        frame_rate_arg,
        flip_image_arg,
        display_arg,
        server_port_arg,
        camera_node,
        web_video_server_node
    ])
