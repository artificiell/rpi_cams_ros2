#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    resolution = LaunchConfiguration('resolution')
    frame_rate = LaunchConfiguration('frame_rate')
    flip_image = LaunchConfiguration('flip_image')
    server_port = LaunchConfiguration('server_port')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'rpi'
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value = 'VGA'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value = '15'
    )
    flip_image_arg = DeclareLaunchArgument(
        'flip_image',
        default_value = 'True'
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
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'framerate': LaunchConfiguration('frame_rate'),
            'codec': 'mjpeg',
            'flip': LaunchConfiguration('flip_image'),
        }]
    )

    # Web server node
    web_sever_node = Node(
        package = 'img_viewer',
        namespace = robot_ns,
        executable = 'web_video_server',
        name = 'camera_web_stream_bridge',
        parameters=[{
            'http_port': LaunchConfiguration('server_port'),
        }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        resolution_arg,
        frame_rate_arg,
        flip_image_arg,
        server_port_arg,
        camera_node,
        web_sever_node
    ])
