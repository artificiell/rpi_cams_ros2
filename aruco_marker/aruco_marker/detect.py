#!/usr/bin/env python3
"""
This node calls the RPi camera capture service (cam_interfaces/srv/Image) to
obtain frames, locates Aruco AR markers in each one, and publishes their ids
and poses.

Service client:
    <service_name> (cam_interfaces.srv.Image), default "camera/image"

Published Topics:
    aruco/markers/original (aruco_interfaces.msg.ArucoMarkers)
       Array of all detected marker ids and poses, relative to the camera.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_ARUCO_ORIGINAL)
    service_name - camera capture service to call (default "camera/image")
    camera_frame - camera optical frame to use (default: taken from the
                   capture service's response info if left blank)

"""

import rclpy
from rclpy.node import Node
from packaging.version import Version
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from cam_interfaces.srv import Image
from cam_interfaces.msg import ArucoMarker, ArucoMarkers
from geometry_msgs.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoDetection(Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )
        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_ARUCO_ORIGINAL",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )
        self.declare_parameter(
            name="service_name",
            value="camera/image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera capture service to call (cam_interfaces/srv/Image).",
            ),
        )
        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")
        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")
        service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera capture service: {service_name}")
        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Make sure we have a valid dictionary id.
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(
                f"bad aruco_dictionary_id: {dictionary_id_name}\nvalid options: {options}"
            )
            raise

        # Set up aruco detector (handles both pre- and post-4.7 OpenCV APIs)
        if Version(cv2.__version__) > Version("4.7.0"):
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            aruco_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        else:
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        # Set up publisher
        self.markers_pub = self.create_publisher(
            ArucoMarkers, "aruco/markers", 10
        )

        # Set up the camera capture service client.
        self.capture_client = self.create_client(Image, service_name)
        while not self.capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Waiting for capture service '{service_name}'...")

        # Kick off the request chain: as soon as one capture images has 
        # been processed, the next one is requested, keeping the the loop
        # running at whatever rate the capture service can sustain.
        self.request_image_capture()

    def request_image_capture(self):
        request = Image.Request()
        future = self.capture_client.call_async(request)
        future.add_done_callback(self.handle_image_response)

    def handle_image_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn("Capture service reported failure, skipping frame.")
            else:
                try:
                    self.process_frame(response.image, response.info)
                except Exception as e:
                    self.get_logger().error(f"Error processing frame: {e}")
        except Exception as e:
            self.get_logger().error(f"Capture service call failed: {e}")

        # Always request the next frame
        self.request_image_capture() 
        
    def process_frame(self, img_msg, info_msg):
        intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        distortion = np.array(info_msg.d)

        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        markers = ArucoMarkers()
        markers.markers = []
        markers.header.frame_id = (
            self.camera_frame if self.camera_frame else info_msg.header.frame_id
        )
        markers.header.stamp = img_msg.header.stamp

        # Detect markers
        if Version(cv2.__version__) > Version("4.7.0"):
            corners, marker_ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dictionary, parameters=self.aruco_parameters
            )

        # Process each detected marker
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, intrinsic_mat, distortion
            )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                marker = ArucoMarker()
                marker.id = int(marker_id[0])
                marker.pose = pose
                markers.markers.append(marker)

        # Publish markers
        self.markers_pub.publish(markers)
        

# Main function
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
