#!/usr/bin/env python3
"""
Simulates laser scan readings from color segmentation of the camera feed:
detects red obstacles in each incoming image and projects them onto the
ground plane (assuming a known, fixed camera height) to produce a
sensor_msgs/LaserScan.

Subscribes to the camera node's fixed topics:
    camera/image (sensor_msgs.msg.Image)
    camera/info  (sensor_msgs.msg.CameraInfo)

Published Topics:
    scan (sensor_msgs.msg.LaserScan)

Parameters:
    camera_height   - camera height above the floor, in meters (default 0.10)
    camera_offset   - how far forward of the robot's reference frame (e.g. base_link) the camera is mounted, in meters (default 0.04)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import math
import cv2


class RPiImageToLaserScan(Node):
    def __init__(self):
        super().__init__('rpi_image_to_laserscan_node')

        # Declare and get parameters
        self.declare_parameter('camera_height', 0.10)          # camera height above the floor (meters)
        self.declare_parameter('camera_offset', 0.04)          # camera forward of robot's reference frame (meters)

        self.camera_height = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_offset = self.get_parameter('camera_offset').get_parameter_value().double_value

        image_topic = 'camera/image'
        camera_info_topic = 'camera/info'

        # Camera model: populated once the first CameraInfo message arrives
        self.camera_model = PinholeCameraModel()
        self.have_camera_info = False

        # Laser scan parameters
        self.fov = math.radians(60)  # ~60 degrees
        self.angle_min = -self.fov / 2
        self.angle_max = self.fov / 2
        self.num_ranges = 120
        self.range_min = 0.01
        self.range_max = 3.00
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_ranges

        self.bridge = CvBridge()

        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            qos_profile_sensor_data
        )

        # Subscribe with the same QoS the camera publisher uses (best-effort, shallow queue)
        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Subscribed to '{image_topic}' and '{camera_info_topic}'"
        )

     # Callback for reciving camera info     
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_model.fromCameraInfo(msg)
        self.have_camera_info = True

    # Callback for reciving camera images
    def image_callback(self, msg: Image):
        if not self.have_camera_info:
            self.get_logger().warn('No CameraInfo received yet, skipping frame.', throttle_duration_sec=5.0)
        else:

            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.process_image(image, msg.header.stamp)
            except Exception as e:
                self.get_logger().error(f"CvBridge error: {e}")
        

    # Detect obstacles (red strips on the floor) and publish simulated laser scan
    def process_image(self, image, stamp):
        # Convert to HSV for color thresholding
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red in HSV (red wraps around 0/180)
        lower_th1 = np.array([0, 100, 100])
        upper_th1 = np.array([10, 255, 255])
        lower_th2 = np.array([160, 100, 100])
        upper_th2 = np.array([180, 255, 255])
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_th1, upper_th1),
            cv2.inRange(hsv, lower_th2, upper_th2)
        )

        # Find pixel coordinates
        v_coords, u_coords = np.where(mask > 0)

        if len(v_coords) == 0:
            self.publish_empty_scan(stamp)
            return

        cx, cy = self.camera_model.cx(), self.camera_model.cy()
        fx, fy = self.camera_model.fx(), self.camera_model.fy()

        x_opt = (u_coords - cx) / fx
        y_opt = -(v_coords - cy) / fy
        z_opt = np.ones_like(x_opt)

        rays_opt = np.column_stack((x_opt, y_opt, z_opt))  # Shape: (N, 3)

        valid_downward = rays_opt[:, 1] < -1e-5
        if not np.any(valid_downward):
            self.publish_empty_scan(stamp)
            return

        rays_opt = rays_opt[valid_downward]

        t = -self.camera_height / rays_opt[:, 1]
        pts_opt = rays_opt * t[:, np.newaxis]

        # Translate from the camera's optical frame into the robot's reference frame (e.g. base_link)
        pts_opt[:, 2] += self.camera_offset

        distances = np.linalg.norm(pts_opt, axis=1)
        angles = np.arctan2(pts_opt[:, 0], pts_opt[:, 2])
        valid_mask = (
            (angles >= self.angle_min) &
            (angles <= self.angle_max) &
            (distances <= self.range_max) &
            (distances >= self.range_min)
        )

        if not np.any(valid_mask):
            self.publish_empty_scan(stamp)
            return

        valid_angles = angles[valid_mask]
        valid_distances = distances[valid_mask]

        indices = ((valid_angles - self.angle_min) / self.angle_increment).astype(np.int32)
        indices = np.clip(indices, 0, self.num_ranges - 1)

        ranges = np.full(self.num_ranges, np.inf, dtype=np.float32)
        np.minimum.at(ranges, indices, valid_distances)
        ranges = np.flip(ranges, axis=0)

        # Publish laser scan
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = stamp
        scan.header.frame_id = self.camera_model.tfFrame()
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.scan_pub.publish(scan)

    def publish_empty_scan(self, stamp):
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = stamp
        scan.header.frame_id = self.camera_model.tfFrame()
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float('inf')] * self.num_ranges
        self.scan_pub.publish(scan)


# Main function
def main(args=None):
    rclpy.init(args=args)
    node = RPiImageToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
