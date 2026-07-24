#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configuration
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    flip = LaunchConfiguration('flip')
    timeout = LaunchConfiguration('timeout')
    sharpness = LaunchConfiguration('sharpness')
    denoise = LaunchConfiguration('denoise')
    frame_id = LaunchConfiguration('frame_id')
    camera_name = LaunchConfiguration('camera_name')
    service_name = LaunchConfiguration('service_name')
    shutter = LaunchConfiguration('shutter')
    gain = LaunchConfiguration('gain')
    ev = LaunchConfiguration('ev')
    metering = LaunchConfiguration('metering')
    awb = LaunchConfiguration('awb')
    brightness = LaunchConfiguration('brightness')
    contrast = LaunchConfiguration('contrast')

    # Launch arguments — defaults mirror the node's current defaults exactly
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='2028'
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='1520'
    )
    flip_arg = DeclareLaunchArgument(
        'flip',
        default_value='true'
    )
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='2000',
        description='AE/AWB settle time (ms), applied once at startup.'
    )
    sharpness_arg = DeclareLaunchArgument(
        'sharpness',
        default_value='1.5'
    )
    denoise_arg = DeclareLaunchArgument(
        'denoise',
        default_value='cdn_hq',
        description='off / cdn_off, cdn_fast, cdn_hq, auto'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_frame'
    )
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='rpi_camera',
        description='Used to match the calibration file (camera_name: field).'
    )
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/camera/image'
    )
    shutter_arg = DeclareLaunchArgument(
        'shutter',
        default_value='160000',
        description='ExposureTime in microseconds, 0 = auto.'
    )
    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='16.0',
        description='AnalogueGain, 0.0 = auto.'
    )
    ev_arg = DeclareLaunchArgument(
        'ev',
        default_value='2.0',
        description='ExposureValue in stops, 0.0 = none.'
    )
    metering_arg = DeclareLaunchArgument(
        'metering',
        default_value='centre',
        description='centre, spot, average, custom'
    )
    awb_arg = DeclareLaunchArgument(
        'awb',
        default_value='auto',
        description='auto, indoor, tungsten, fluorescent, daylight'
    )
    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='0.0'
    )
    contrast_arg = DeclareLaunchArgument(
        'contrast',
        default_value='1.0'
    )

    # Global Shutter camera node
    gs_camera_node = Node(
        package='rpi_gs_cam',
        executable='camera',
        name='rpi_gs_camera_service',
        parameters=[{
            'width': width,
            'height': height,
            'flip': flip,
            'timeout': timeout,
            'sharpness': sharpness,
            'denoise': denoise,
            'frame_id': frame_id,
            'camera_name': camera_name,
            'service_name': service_name,
            'shutter': shutter,
            'gain': gain,
            'ev': ev,
            'metering': metering,
            'awb': awb,
            'brightness': brightness,
            'contrast': contrast,
        }]
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        flip_arg,
        timeout_arg,
        sharpness_arg,
        denoise_arg,
        frame_id_arg,
        camera_name_arg,
        service_name_arg,
        shutter_arg,
        gain_arg,
        ev_arg,
        metering_arg,
        awb_arg,
        brightness_arg,
        contrast_arg,
        gs_camera_node,
    ])
