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
    camera_height = LaunchConfiguration('camera_height')

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
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value = '0.08'
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

    # Image to laser scan node
    image2scan_node = Node(
        package = 'rpi_cam',
        namespace = robot_ns,
        executable = 'image2scan',
        name = 'rpi_image_to_laserscan_node',
        parameters=[{
            'height': camera_height,
        }]
    )
    
    return LaunchDescription([
        robot_ns_launch_arg,
        resolution_arg,
        frame_rate_arg,
        flip_image_arg,
        camera_height_arg,
        camera_node,
        image2scan_node
    ])
