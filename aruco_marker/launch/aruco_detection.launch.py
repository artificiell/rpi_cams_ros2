#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    resolution = LaunchConfiguration('resolution')
    frame_rate = LaunchConfiguration('frame_rate')
    flip_image = LaunchConfiguration('flip_image')
    marker_size = LaunchConfiguration('marker_size')
    aruco_dictionary_id = LaunchConfiguration('aruco_dictionary_id')
    camera_frame = LaunchConfiguration('camera_frame')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value='rpi'
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='VGA'
    )
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='15'
    )
    flip_image_arg = DeclareLaunchArgument(
        'flip_image',
        default_value='True'
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1'
    )
    aruco_dictionary_id_arg = DeclareLaunchArgument(
        'aruco_dictionary_id',
        default_value='DICT_ARUCO_ORIGINAL'
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value=''
    )

    # Include camera capture service
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rpi_cam'), 'launch', 'camera.launch.py'
            )
        ),
        launch_arguments={
            'robot_ns': robot_ns,
            'resolution': resolution,
            'frame_rate': frame_rate,
            'flip_image': flip_image,
        }.items(),
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_marker',
        namespace=robot_ns,
        executable='detect',
        name='aruco_detection_node',
        parameters=[{
            'marker_size': ParameterValue(marker_size, value_type=float),
            'aruco_dictionary_id': aruco_dictionary_id,
            'camera_frame': camera_frame,
            'display': False
        }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        resolution_arg,
        frame_rate_arg,
        flip_image_arg,
        marker_size_arg,
        aruco_dictionary_id_arg,
        camera_frame_arg,
        camera_launch,
        aruco_node,
    ])
