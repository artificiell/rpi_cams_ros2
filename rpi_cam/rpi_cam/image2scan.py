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
        pixels = np.column_stack(np.where(mask > 0))

        # Initialize scan ranges
        ranges = [float('inf')] * self.num_ranges
        angle_increment = (self.angle_max - self.angle_min) / self.num_ranges

        # Project pixels to rays
        for v, u in pixels:
            # Back-project to ray
            x = (u - self.camera_model.cx()) / self.camera_model.fx()
            y = -(v - self.camera_model.cy()) / self.camera_model.fy()
            z = 1.0
            ray = np.array([x, y, z])
            ray = ray / np.linalg.norm(ray)

            if ray[1] >= 0:
                continue  # ray going up

            t = -self.camera_height / ray[1]
            point = t * ray
            distance = np.linalg.norm(point)

            # Angle in image plane (X-Z)
            angle = math.atan2(point[0], point[2])
            if self.angle_min <= angle <= self.angle_max and self.range_min <= distance <= self.range_max:
                index = int((angle - self.angle_min) / angle_increment)
                if 0 <= index < self.num_ranges:
                    if distance < ranges[index]:
                        ranges[index] = distance

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
        scan.ranges = ranges

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
