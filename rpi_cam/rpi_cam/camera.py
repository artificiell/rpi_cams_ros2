#!/usr/bin/env python3
import os
import copy
import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge

from cam_interfaces.srv import Image


# Class for handle RPi Camera Module (v2)
class RPiCameraService(Node):
    def __init__(self):
        super().__init__('rpi_camera_service')

        # Declare and get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('flip', True)
        self.declare_parameter('quality', 95)       # JPEG compression quality (0-100)
        self.declare_parameter('timeout', 100)      # Camera capture timeout (ms)
        self.declare_parameter('sharpness', 1.0)
        self.declare_parameter('denoise', 'auto')   # auto, cdn_off, cdn_fast, cdn_hq
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('camera_name', 'rpi_camera')
        self.frame_width = self.get_parameter('width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('height').get_parameter_value().integer_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.sharpness = self.get_parameter('sharpness').get_parameter_value().double_value
        self.denoise = self.get_parameter('denoise').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value 
        self.get_logger().warn(f"Initalizing camera with resolution: {self.frame_width} x {self.frame_height}")
        self.get_logger().warn(f"Denoise: {self.denoise}")
        
        # Read camera parameters
        calibratrion_file = self.get_calibration_filename(self.frame_width, self.frame_height)
        self.camera_info_template = CameraInfo()
        if calibratrion_file is not None:
            self.get_logger().warn(f"Using calibration file: {calibratrion_file}")
            cname = self.get_parameter('camera_name').get_parameter_value().string_value
            url = os.path.join(get_package_share_directory('rpi_cam'), 'config', calibratrion_file)
            camera_info_manager = CameraInfoManager(self, cname = cname, url = f'file://{url}')
            camera_info_manager.loadCameraInfo()
            self.camera_info_template = camera_info_manager.getCameraInfo()
        else:
            self.get_logger().warn(f"Could not load calibration, using default instead.")        
            self.camera_info_template.width = int(self.frame_width)
            self.camera_info_template.height = int(self.frame_height)
            self.camera_info_template.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Dummy
            self.camera_info_template.p = [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0]  # Dummy
            self.camera_info_template.distortion_model = 'plumb_bob'
            self.camera_info_template.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Set up ROS service
        self.bridge = CvBridge()
        self.service = self.create_service(Image, 'camera/image', self.capture_callback)
        self.get_logger().info('Camera image capture service ready: /camera/image')

    # Map exact image resolution to calibration filenames
    def get_calibration_filename(self, width: int, height: int):
        calibration_map = {
            (320, 180): 'calibration_320x180.yaml',
            (320, 240): 'calibration_320x240.yaml',
            (640, 360): 'calibration_640x360.yaml',
            (640, 480): 'calibration_640x480.yaml',
            #(1640, 1232): 'calibration_1640x1232.yaml',
            #(1920, 1080): 'calibration_1920x1080.yaml',
            #(3280, 2464): 'calibration_3280x2464.yaml',
        }
        return calibration_map.get((width, height), None)

    # Explicit IMX219 camera mode string for known native modes
    def get_native_mode(self, width: int, height: int):
        native_modes = {
            (640, 480): '640:480:10:P',
            (1640, 1232): '1640:1232:10:P',
            (1920, 1080): '1920:1080:10:P',
            (3280, 2464): '3280:2464:10:P',
        }
        return native_modes.get((width, height), None)
    
    # Cature frame from the Raspberry Pi camera
    def capture_callback(self, request: Image.Request, response: Image.Response):
        response.success = False
        try:

            # Capture an image frame
            img = self.capture_still_image()
            self.get_logger().info(f'Image resolution: {img.shape}')
            if self.flip:
                img = cv2.flip(cv2.flip(img, 0), 1)

            # Create and send response messages
            stamp = self.get_clock().now().to_msg()
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding = 'bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            info_msg = self.create_camera_info_message(
                stamp = stamp,
                width = img.shape[1],
                height = img.shape[0]
            ) 
            response.image = image_msg
            response.info = info_msg
            response.success = True

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')
        finally:
            return response
            
    # Capture still image frames using rpicam-still command 
    def capture_still_image(self) -> np.array:
        cmd = [
            'rpicam-still',
            '--width', str(self.frame_width),
            '--height', str(self.frame_height),
            '--timeout', str(self.timeout),
            '--nopreview',
            '--encoding', 'jpg',
            '--quality', str(self.quality),
            '--sharpness', str(self.sharpness),
            '--denoise', self.denoise,
            '--output', '-'
        ]
        native_mode = self.get_native_mode(self.frame_width, self.frame_height)
        if native_mode is not None:
            cmd.extend(['--mode', native_mode])
        

        # Run rpicam-still subprocess
        result = subprocess.run(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        if result.returncode != 0:
            stderr_text = result.stderr.decode(errors = 'ignore').strip()
            raise RuntimeError(
                f'Capturing still image failed with code {result.returncode}: {stderr_text}'
            )
        if not result.stdout:
            raise RuntimeError('Capturing still image returned no image buffer.')

        # Convert frame buffer to NumPy array 
        np_arr = np.frombuffer(result.stdout, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError('Failed to decode JPEG image from frame image buffer.')
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
def main(args = None):
    rclpy.init(args = args)
    node = RPiCameraService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
