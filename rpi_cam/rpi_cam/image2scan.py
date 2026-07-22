#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from cam_interfaces.srv import Image as CameraImageSrv
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
        self.declare_parameter('height', 0.08)        # meters above the floor
        self.declare_parameter('service_name', 'camera/image')
        self.declare_parameter('period', 0.2)         # how often to request a new frame (s)

        # Camera parameters
        self.camera_model = PinholeCameraModel()
        self.camera_height = self.get_parameter('height').get_parameter_value().double_value
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        call_period = self.get_parameter('period').get_parameter_value().double_value

        # Laser scan parameters
        self.fov = math.radians(60)  # ~60 degrees
        self.angle_min = -self.fov / 2
        self.angle_max = self.fov / 2
        self.num_ranges = 120
        self.range_min = 0.01
        self.range_max = 3.00

        # Set up ROS client and publisher
        self.bridge = CvBridge()
        self.client = self.create_client(CameraImageSrv, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for camera service "{service_name}"...')

        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            qos_profile_sensor_data
        )

        # A service is request/response, not a stream — a timer drives the requests
        self.request_in_flight = False
        self.timer = self.create_timer(call_period, self.request_image)

    # Timer callback: ask the camera service for a new image + camera info
    def request_image(self):
        if self.request_in_flight:
            self.get_logger().warn('Previous request still in flight, skipping this cycle.')
        else:
            self.request_in_flight = True
            request = CameraImageSrv.Request()
            # Non-blocking call: never use client.call() here, it would deadlock
            # the node waiting on its own request while spinning.
            future = self.client.call_async(request)
            future.add_done_callback(self.image_response_callback)

    # Callback triggered once the service call completes
    def image_response_callback(self, future):
        self.request_in_flight = False

        response = None
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        if response is not None:
            if not response.success:
                self.get_logger().warn('Camera service reported a failed capture.')
            else:
                # Camera intrinsics now arrive bundled with every response —
                # no need for the old got_info bookkeeping.
                self.camera_model.fromCameraInfo(response.info)

                image = None
                try:
                    image = self.bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')
                except Exception as e:
                    self.get_logger().error(f"CvBridge error: {e}")

                if image is not None:
                    self.process_image(image, response.image.header.stamp)

    # Detect red obstacles in the image and publish the simulated laser scan
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

        # Initialize scan ranges
        angle_increment = (self.angle_max - self.angle_min) / self.num_ranges

        valid_downward = rays_opt[:, 1] < -1e-5
        if not np.any(valid_downward):
            self.publish_empty_scan(stamp)
            return

        rays_opt = rays_opt[valid_downward]

        t = -self.camera_height / rays_opt[:, 1]
        pts_opt = rays_opt * t[:, np.newaxis]

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

        indices = ((valid_angles - self.angle_min) / angle_increment).astype(np.int32)
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
        scan.angle_increment = angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()

        self.scan_pub.publish(scan)

    def publish_empty_scan(self, stamp):
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float('inf')] * self.num_ranges
        self.scan_pub.publish(scan)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = RPiImageToLaserScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
