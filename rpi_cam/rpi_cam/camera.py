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
        self.declare_parameter('resolution', 'VGA') # QVGA, VGA (default), 180p, or 320p
        self.declare_parameter('flip', True)
        self.declare_parameter('quality', 95)       # JPEG compression quality (0-100)
        self.declare_parameter('timeout', 100)      # Camera capture timeout (ms)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('camera_name', 'rpi_camera')        
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value
        self.quality = self.get_parameter('quality').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value 
        
        # Set parameters besed on camera resolution
        if self.get_parameter('resolution').get_parameter_value().string_value.lower() == 'qvga':
            self.frame_width, self.frame_height = 320, 240
            calibratrion_file = 'calibration_qvga.yaml'
        elif self.get_parameter('resolution').get_parameter_value().string_value.lower() == '180p':
            self.frame_width, self.frame_height = 320, 180
            calibratrion_file = 'calibration_180p.yaml'
        elif self.get_parameter('resolution').get_parameter_value().string_value.lower() == '360p':
            self.frame_width, self.frame_height = 640, 360
            calibratrion_file = 'calibration_360p.yaml'
        else:
            self.frame_width, self.frame_height = 640, 480
            calibratrion_file = 'calibration_vga.yaml'
        self.get_logger().warn(f"Initalizing camera with resolution {self.frame_width} x {self.frame_height}")
        
        # Read camera parameters
        self.camera_info_msg = CameraInfo()
        try:
            cname = self.get_parameter('camera_name').get_parameter_value().string_value
            url = os.path.join(get_package_share_directory('rpi_cam'), 'config', calibratrion_file)
            camera_info_manager = CameraInfoManager(self, cname = cname, url = f'file://{url}')
            camera_info_manager.loadCameraInfo()
            self.camera_info_template = camera_info_manager.getCameraInfo()
        except Exception as e:
            self.get_logger().warn(f"Could not load calibration, using default instead.")        
            self.camera_info_msg.width = int(self.width)
            self.camera_info_msg.height = int(self.height)
            self.camera_info_msg.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Dummy
            self.camera_info_msg.p = [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0]  # Dummy
            self.camera_info_msg.distortion_model = 'plumb_bob'
            self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Set up ROS service
        self.bridge = CvBridge()
        self.service = self.create_service(Image, 'camera/image', self.capture_callback)
        self.get_logger().info('Camera image capture service ready: /camera/image')

    # Cature frame from the Raspberry Pi camera
    def capture_callback(self, request: Image.Request, response: Image.Response) -> None:
        response.success = False
        try:

            # Capture an image frame
            img = self.capture_still_image()
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
            response.success = False

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
            '--output', '-'
        ]

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
