#!/usr/bin/env python3
import copy
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge

from picamera2 import Picamera2

from cam_interfaces.srv import Image


# Class for handling RPi Camera Module (v2) via picamera2 (persistent camera,
# no per-call subprocess). Mirrors camera.py's interface/params so the two
# nodes can be A/B tested side by side.
class RPiCamera2Service(Node):
    def __init__(self):
        super().__init__('rpi_camera2_service')

        # Declare and get parameters (same names/defaults as camera.py where applicable)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('flip', True)
        self.declare_parameter('warmup_frames', 10)  # frames to discard while AE/AWB converge at startup
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('camera_name', 'rpi_camera')
        self.declare_parameter('service_name', 'camera/image')
        self.frame_width = self.get_parameter('width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('height').get_parameter_value().integer_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value
        self.warmup_frames = self.get_parameter('warmup_frames').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.get_logger().warn(f"Initializing camera (picamera2) with resolution: {self.frame_width} x {self.frame_height}")

        # Read camera parameters (same calibration lookup as camera.py)
        calibratrion_file = self.get_calibration_filename(self.frame_width, self.frame_height)
        self.camera_info_template = CameraInfo()
        if calibratrion_file is not None:
            self.get_logger().warn(f"Using calibration file: {calibratrion_file}")
            cname = self.get_parameter('camera_name').get_parameter_value().string_value
            url = os.path.join(get_package_share_directory('rpi_cam'), 'config', calibratrion_file)
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

        # --- Persistent camera setup ---
        # The camera is opened once and kept running for the node's lifetime.
        # capture_array() then just grabs the latest frame from the already
        # running pipeline, instead of spawning rpicam-still per request.
        self._cam_lock = threading.Lock()
        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (self.frame_width, self.frame_height), "format": "BGR888"}
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        self.get_logger().info(f'Warming up camera ({self.warmup_frames} frames for AE/AWB convergence)...')
        for _ in range(self.warmup_frames):
            self.picam2.capture_array()
        self.get_logger().info('Camera ready.')

        # Set up ROS service
        self.bridge = CvBridge()
        self.service = self.create_service(Image, self.service_name, self.capture_callback)
        self.get_logger().info(f'Camera2 (picamera2) image capture service ready: {self.service_name}')

    def destroy_node(self):
        try:
            self.picam2.stop()
            self.picam2.close()
        except Exception:
            pass
        super().destroy_node()

    # Map exact image resolution to calibration filenames
    def get_calibration_filename(self, width: int, height: int):
        calibration_map = {
            (320, 180): 'calibration_320x180.yaml',
            (320, 240): 'calibration_320x240.yaml',
            (640, 360): 'calibration_640x360.yaml',
            (640, 480): 'calibration_640x480.yaml',
        }
        return calibration_map.get((width, height), None)

    # Capture frame from the Raspberry Pi camera
    def capture_callback(self, request: Image.Request, response: Image.Response):
        response.success = False
        try:
            img = self.capture_still_image()
            if self.flip:
                img = cv2.flip(cv2.flip(img, 0), 1)

            stamp = self.get_clock().now().to_msg()
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            info_msg = self.create_camera_info_message(
                stamp=stamp,
                width=img.shape[1],
                height=img.shape[0]
            )
            response.image = image_msg
            response.info = info_msg
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')
        finally:
            return response

    # Grab the latest frame straight from the already-running camera pipeline.
    # No process spawn, no pipeline re-init, no JPEG encode/decode round trip.
    def capture_still_image(self) -> np.array:
        with self._cam_lock:
            frame = self.picam2.capture_array()
        if frame is None:
            raise RuntimeError('Failed to capture frame from camera.')
        if frame.ndim == 3 and frame.shape[2] == 4:
            # Defensive: fall back to dropping the alpha/padding channel if a
            # different picamera2 config ever returns XBGR8888 instead of BGR888.
            frame = frame[:, :, :3]
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Correct for rpicam-still channel order bug
        return frame

    # Create CameraInfo messages
    def create_camera_info_message(self, stamp, width: int, height: int) -> CameraInfo:
        msg = copy.deepcopy(self.camera_info_template)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        return msg


# Main function
def main(args=None):
    rclpy.init(args=args)
    node = RPiCamera2Service()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
