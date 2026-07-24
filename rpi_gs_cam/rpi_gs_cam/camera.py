#!/usr/bin/env python3
import os
import copy
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult

from picamera2 import Picamera2
from libcamera import controls as libcontrols

from cam_interfaces.srv import Image


# Class for handling the Raspberry Pi Global Shutter Camera via picamera2
class RPiGSCameraService(Node):

    # Parameters that require a camera reconfigure, which can not be changes at runtime (restart required)
    STRUCTURAL_PARAMS = {'width', 'height', 'camera_name', 'service_name', 'frame_id', 'timeout'}

    # Parameters that map directly onto a picamera2/libcamera control, which can directly be changed at runtime
    ADJUSTABLE_CONTROL_PARAMS = {
        'shutter', 'gain', 'ev', 'metering', 'awb', 'brightness', 'contrast', 'sharpness', 'denoise'
    }

    def __init__(self):
        super().__init__('rpi_gs_camera_service')

        # Declare and get parameters
        self.declare_parameter('width', 2028)
        self.declare_parameter('height', 1520)
        self.declare_parameter('flip', True)
        self.declare_parameter('timeout', 2000)         # AE/AWB settle time (ms), applied ONCE at startup
        self.declare_parameter('sharpness', 1.5)
        self.declare_parameter('denoise', 'cdn_hq')     # off/cdn_off, cdn_fast, cdn_hq, auto
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('camera_name', 'rpi_camera')
        self.declare_parameter('service_name', '/camera/image')

        self.declare_parameter('shutter', 160000)       # ExposureTime (us), 0 = auto
        self.declare_parameter('gain', 16.0)            # AnalogueGain, 0.0 = auto
        self.declare_parameter('ev', 2.0)               # ExposureValue (stops), 0.0 = none
        self.declare_parameter('metering', 'centre')    # centre, spot, average, custom
        self.declare_parameter('awb', 'auto')           # auto, indoor, tungsten, fluorescent, daylight
        self.declare_parameter('brightness', 0.0)       # -1.0 to 1.0
        self.declare_parameter('contrast', 1.0)         # 0.0 to 32.0

        # Get all parameters
        self.frame_width = self.get_parameter('width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('height').get_parameter_value().integer_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.sharpness = self.get_parameter('sharpness').get_parameter_value().double_value
        self.denoise = self.get_parameter('denoise').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.shutter = self.get_parameter('shutter').get_parameter_value().integer_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.ev = self.get_parameter('ev').get_parameter_value().double_value
        self.metering = self.get_parameter('metering').get_parameter_value().string_value
        self.awb = self.get_parameter('awb').get_parameter_value().string_value
        self.brightness = self.get_parameter('brightness').get_parameter_value().double_value
        self.contrast = self.get_parameter('contrast').get_parameter_value().double_value

        self.get_logger().warn(f"Initializing camera (picamera2) with resolution: {self.frame_width} x {self.frame_height}")
        self.get_logger().warn(f"Denoise: {self.denoise}, Metering: {self.metering}, AWB: {self.awb}")

        # Read camera calibration
        calibratrion_file = self.get_calibration_filename(self.frame_width, self.frame_height)
        self.camera_info_template = CameraInfo()
        if calibratrion_file is not None:
            self.get_logger().warn(f"Using calibration file: {calibratrion_file}")
            cname = self.get_parameter('camera_name').get_parameter_value().string_value
            url = os.path.join(get_package_share_directory('rpi_gs_cam'), 'config', calibratrion_file)
            camera_info_manager = CameraInfoManager(self, cname=cname, url=f'file://{url}')
            camera_info_manager.loadCameraInfo()
            self.camera_info_template = camera_info_manager.getCameraInfo()
        else:
            self.get_logger().warn("Could not load calibration, using default instead.")
            self.camera_info_template.width = int(self.frame_width)
            self.camera_info_template.height = int(self.frame_height)
            self.camera_info_template.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Dummy
            self.camera_info_template.p = [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0]  # Dummy
            self.camera_info_template.distortion_model = 'plumb_bob'
            self.camera_info_template.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Persistent camera instance
        self._cam_lock = threading.Lock()
        self.picam2 = Picamera2()

        controls = self.build_controls()
        video_config = self.picam2.create_still_configuration(
            main={"size": (self.frame_width, self.frame_height), "format": "BGR888"},
            controls=controls,
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        # One-time settle delay for AE/AWB to converge
        settle_seconds = self.timeout / 1000.0
        self.get_logger().info(f'Warming up camera ({settle_seconds:.2f}s for AE/AWB convergence)...')
        time.sleep(settle_seconds)
        self.get_logger().info('Camera ready.')

        # Set up ROS service
        self.bridge = CvBridge()
        self.service = self.create_service(Image, self.service_name, self.capture_callback)
        self.get_logger().info(f'Camera image capture service ready: {self.service_name}')

        # Allow the essential parameters (such as exposure/image) to be tuned at
        self.add_on_set_parameters_callback(self.on_parameter_change)


    # Handle updates of ROS parameters at runtime
    def on_parameter_change(self, params):

        # Reject the whole batch if it contains any structural parameters
        for param in params:
            if param.name in self.STRUCTURAL_PARAMS:
                return SetParametersResult(
                    successful = False,
                    reason = f"'{param.name}' can't be changed at runtime — restart the node to apply a new value."
                )

        control_changed = False
        for param in params:
            if param.name == 'flip':
                self.flip = param.value
            elif param.name in self.ADJUSTABLE_CONTROL_PARAMS:
                setattr(self, param.name, param.value)
                control_changed = True

        if control_changed:
            try:
                new_controls = self.build_controls()
                self.picam2.set_controls(new_controls)
                self.get_logger().info('Applied updated camera controls.')
            except Exception as e:
                return SetParametersResult(successful = False, reason = f'Failed to apply updated controls: {e}')

        return SetParametersResult(successful = True)

    # Build picamera2/libcamera control dict from ROS parameters
    def build_controls(self) -> dict:
        controls = {
            'Sharpness': self.sharpness,
            'Brightness': self.brightness,
            'Contrast': self.contrast,
            'AwbEnable': True,
        }

        # Exposure: 0 / 0.0 means "leave on auto"
        if self.shutter > 0:
            controls['ExposureTime'] = self.shutter
        if self.gain > 0.0:
            controls['AnalogueGain'] = self.gain
        if self.ev != 0.0:
            controls['ExposureValue'] = self.ev

        # Metering mode
        metering_map = {
            'centre': libcontrols.AeMeteringModeEnum.CentreWeighted,
            'spot': libcontrols.AeMeteringModeEnum.Spot,
            'average': libcontrols.AeMeteringModeEnum.Matrix,
            'custom': libcontrols.AeMeteringModeEnum.Custom,
        }
        if self.metering in metering_map:
            controls['AeMeteringMode'] = metering_map[self.metering]
        else:
            self.get_logger().warn(f"Unknown metering mode '{self.metering}', using 'centre'.")
            controls['AeMeteringMode'] = metering_map['centre']

        # White balance preset
        awb_map = {
            'auto': libcontrols.AwbModeEnum.Auto,
            'indoor': libcontrols.AwbModeEnum.Indoor,
            'tungsten': libcontrols.AwbModeEnum.Tungsten,
            'fluorescent': libcontrols.AwbModeEnum.Fluorescent,
            'daylight': libcontrols.AwbModeEnum.Daylight,
        }
        if self.awb in awb_map:
            controls['AwbMode'] = awb_map[self.awb]
        else:
            self.get_logger().warn(f"Unknown AWB mode '{self.awb}', using 'auto'.")
            controls['AwbMode'] = awb_map['auto']

        # Noise reduction 
        denoise_map = {
            'off': libcontrols.draft.NoiseReductionModeEnum.Off,
            'cdn_off': libcontrols.draft.NoiseReductionModeEnum.Off,
            'cdn_fast': libcontrols.draft.NoiseReductionModeEnum.Fast,
            'cdn_hq': libcontrols.draft.NoiseReductionModeEnum.HighQuality,
            'auto': libcontrols.draft.NoiseReductionModeEnum.Fast,
        }
        if self.denoise in denoise_map:
            controls['NoiseReductionMode'] = denoise_map[self.denoise]
        else:
            self.get_logger().warn(f"Unknown denoise mode '{self.denoise}', using 'cdn_hq'.")
            controls['NoiseReductionMode'] = denoise_map['cdn_hq']

        return controls

    # Map exact image resolution to calibration filenames
    def get_calibration_filename(self, width: int, height: int):
        calibration_map = {
            (640, 480): 'calibration_640x480.yaml',
            (2028, 1520): 'calibration_2028x1520.yaml',
            # (1640, 1232): 'calibration_1640x1232.yaml',
            # (1920, 1080): 'calibration_1920x1080.yaml',
        }
        return calibration_map.get((width, height), None)

    # Capture frame from the Raspberry Pi camera
    def capture_callback(self, request: Image.Request, response: Image.Response):
        response.success = False
        try:
            img = self.capture_image()
            self.get_logger().info(f'Image resolution: {img.shape}')
            if self.flip:
                img = cv2.flip(cv2.flip(img, 0), 1)

            stamp = self.get_clock().now().to_msg()
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            info_msg = self.create_camera_info_message(
                stamp=stamp,
                width=img.shape[1],
                height=img.shape[0],
            )
            response.image = image_msg
            response.info = info_msg
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')
        finally:
            return response

    # Capture the latest frame from the camera pipeline.
    def capture_image(self) -> np.array:
        with self._cam_lock:
            frame = self.picam2.capture_array()
            if frame is None:
                raise RuntimeError('Failed to capture frame from camera.')

            # Defensive: fall back to dropping the alpha/padding channel if a
            # different picamera2 config ever returns XBGR8888 instead of BGR888.
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Correct for known channel-order quirk
            return frame

    # Create CameraInfo messages
    def create_camera_info_message(self, stamp, width: int, height: int) -> CameraInfo:
        msg = copy.deepcopy(self.camera_info_template)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        return msg

    # Ensure that the camera is released
    def destroy_node(self):
        try:
            self.picam2.stop()
            self.picam2.close()
        except Exception:
            pass
        super().destroy_node()

        
# Main function
def main(args=None):
    rclpy.init(args=args)
    node = RPiGSCameraService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
