#!/usr/bin/env python3
"""
Publishes images from the RPi Camera Module (v2) via picamera2 (persistent
camera, no per-call subprocess) on standard ROS 2 topics, at a fixed rate.

Published Topics:
    camera/image      (sensor_msgs.msg.Image)
    camera/info       (sensor_msgs.msg.CameraInfo)

Parameters:
    width, height     - capture resolution (default 640x480)
    flip              - whether to flip the image (default True)
    warmup_frames     - frames to discard while AE/AWB converge at startup (default 10)
    publish_rate      - publish rate in Hz (default 30.0)
    frame_id          - camera optical frame (default "camera_frame")
    camera_name       - name used to look up calibration (default "rpi_camera")
    image_topic       - default "camera/image_raw"
    camera_info_topic - default "camera/camera_info"

QoS: images are published BEST_EFFORT with a shallow queue (depth 1) — the
standard choice for image topics, so a slow subscriber just gets the next
new frame rather than the publisher blocking on a backlog.
"""

import copy
import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge

from picamera2 import Picamera2

# Wrapper class for RPi camera
class RPiCameraPublisher(Node):
    def __init__(self):
        super().__init__('rpi_camera_publisher')

        # Declare and get parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('flip', True)
        self.declare_parameter('warmup_frames', 10)
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('camera_name', 'rpi_camera')
        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('camera_info_topic', 'camera/info')

        self.frame_width = self.get_parameter('width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('height').get_parameter_value().integer_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value
        self.warmup_frames = self.get_parameter('warmup_frames').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.get_logger().info(
            f"Initializing camera (picamera2) with resolution: {self.frame_width} x {self.frame_height}"
        )
        self.get_logger().info(f"Publishing images on topic '{image_topic}' at {self.frame_rate} Hz")

        # Read camera calibration
        calibratrion_file = self.get_calibration_filename(self.frame_width, self.frame_height)
        self.camera_info_template = CameraInfo()
        if calibratrion_file is not None:
            self.get_logger().info(f"Using calibration file: {calibratrion_file}")
            cname = self.get_parameter('camera_name').get_parameter_value().string_value
            url = os.path.join(get_package_share_directory('rpi_cam'), 'config', calibratrion_file)
            camera_info_manager = CameraInfoManager(self, cname=cname, url=f'file://{url}')
            camera_info_manager.loadCameraInfo()
            self.camera_info_template = camera_info_manager.getCameraInfo()
        else:
            self.get_logger().warn("Could not load calibration, using default instead.")
            self.camera_info_template.width = int(self.frame_width)
            self.camera_info_template.height = int(self.frame_height)
            self.camera_info_template.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]           # Dummy
            self.camera_info_template.p = [1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0]  # Dummy
            self.camera_info_template.distortion_model = 'plumb_bob'
            self.camera_info_template.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Persistent camera instance 
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

        self.bridge = CvBridge()

        # QoS for image topics: reliable, shallow queue (also works for best-effort subscribers)
        image_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, 
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1,
        )

        # ROS publisher and subscribers
        self.image_pub = self.create_publisher(Image, image_topic, image_qos)
        self.info_pub = self.create_publisher(CameraInfo, camera_info_topic, image_qos)
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.publish_frame)
        self.get_logger().info(f'Camera publisher ready: {image_topic}')


    # Map exact image resolution to calibration filenames
    def get_calibration_filename(self, width: int, height: int):
        calibration_map = {
            (320, 180): 'calibration_320x180.yaml',
            (320, 240): 'calibration_320x240.yaml',
            (640, 360): 'calibration_640x360.yaml',
            (640, 480): 'calibration_640x480.yaml',
        }
        return calibration_map.get((width, height), None)

    # Publish images at fixed rate
    def publish_frame(self):
        try:
            img = self.grab_image()
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

            self.image_pub.publish(image_msg)
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to capture/publish image: {e}')

    # Grab the latest frame straight from the already-running camera pipeline
    def grab_image(self) -> np.array:
        with self._cam_lock:
            frame = self.picam2.capture_array()
            if frame is None:
                raise RuntimeError('Failed to capture frame from camera.')
            
            # Defensive: fall back to dropping the alpha/padding channel if a
            # different picamera2 config ever returns XBGR8888 instead of BGR888.
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]

            # Correct for rpicam-still channel order bug
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  
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
    node = RPiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
