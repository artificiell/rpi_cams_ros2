#!/usr/bin/env python3
"""
Subscribes to the camera node's fixed topics (camera/image, camera/info),
locates ArUco AR markers in each incoming frame, and:

  1. Publishes their ids and poses on a topic, at the same rate frames
     arrive. This is small structured data, so it doesn't run into the
     bandwidth/stability issues that motivated moving images to services in
     an earlier iteration of this system, and it is ALWAYS published
     regardless of the "display" setting below.
  2. Optionally publishes the annotated frame (markers + axes drawn on top
     of the raw frame) on the "aruco/image" topic. This publisher is only
     created (once, at startup) while the "display" parameter is true.

Subscribed Topics:
    camera/image (sensor_msgs.msg.Image)
    camera/info  (sensor_msgs.msg.CameraInfo)

Published Topics:
    aruco/markers (cam_interfaces.msg.ArucoMarkers)
       Array of all detected marker ids and poses, relative to the camera.
       Published continuously, independent of "display".

    aruco/image (sensor_msgs.msg.Image)
       The annotated frame. Only published while "display" == True.

Parameters:
    marker_size          - size of the markers in meters (default 0.1)
    aruco_dictionary_id  - dictionary used to generate markers
                            (default DICT_ARUCO_ORIGINAL)
    display               - whether to publish the "aruco/image" topic
                            (default True). Read once at startup.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from packaging.version import Version
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo

from cam_interfaces.msg import ArucoMarker, ArucoMarkers

# Class for detecting ArUco markers
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
            name="display",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to publish the annotated-image topic ('aruco/image').",
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

        self.display = self.get_parameter("display").get_parameter_value().bool_value
        self.get_logger().info(f"Display ('aruco/image' topic) enabled: {self.display}")

        # Make sure we have a valid dictionary id.
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error(
                f"Bad aruco_dictionary_id: {dictionary_id_name}\nvalid options: {options}"
            )
            raise

        # Set up ArUco detector (handles both pre- and post-4.7 OpenCV APIs)
        if Version(cv2.__version__) > Version("4.7.0"):
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            aruco_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        else:
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        # Camera model info: cached from the latest CameraInfo message.
        self.latest_info = None

        # Publisher for marker ids/poses (always)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco/markers", 10)

        # Publish annotated image (only if display == True)
        self.aruco_image_pub = None
        if self.display:

            # QoS for image topics: reliable, shallow queue (also works for best-effort subscribers)
            image_qos = QoSProfile(
                reliability = QoSReliabilityPolicy.RELIABLE, 
                history = QoSHistoryPolicy.KEEP_LAST,
                depth = 1,
            )
            self.aruco_image_pub = self.create_publisher(Image, "aruco/image", image_qos)
            self.get_logger().info("Display enabled: publishing 'aruco/image'.")

        # Subscribers for camera image and info
        self.info_sub = self.create_subscription(
            CameraInfo, "camera/info", self.camera_info_callback, qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, "camera/image", self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to 'camera/image' and 'camera/info'")


    # Callback for reciving camera info
    def camera_info_callback(self, msg: CameraInfo):
        self.latest_info = msg

    # Callback for reciving camera images
    def image_callback(self, msg: Image):
        if self.latest_info is None:
            self.get_logger().warn("No CameraInfo received yet, skipping frame.", throttle_duration_sec=5.0)
        else:
            try:
                self.process_and_publish(msg, self.latest_info)
            except Exception as e:
                self.get_logger().error(f"Error processing frame: {e}")

    # Process image frame, publish poses, and (if enabled) publish the annotated frame
    def process_and_publish(self, img_msg, info_msg):
        annotated, markers = self.detect_markers(img_msg, info_msg)
        markers_msg = ArucoMarkers()
        markers_msg.header = img_msg.header
        markers_msg.markers = markers
        self.markers_pub.publish(markers_msg)
        if self.display and self.aruco_image_pub is not None:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = img_msg.header
            self.aruco_image_pub.publish(annotated_msg)

    # Detect markers and create annotated image
    def detect_markers(self, img_msg, info_msg):
        intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        distortion = np.array(info_msg.d)
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        annotated = image.copy() if self.display else None
        markers = []

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
            if self.display:
                cv2.aruco.drawDetectedMarkers(annotated, corners, marker_ids)

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
                markers.append(marker)

                # Draw marker information
                if self.display:
                    cv2.drawFrameAxes(
                        annotated, intrinsic_mat, distortion,
                        rvecs[i], tvecs[i], self.marker_size * 0.5
                    )
                    text_pos = tuple(corners[i][0][0].astype(int))
                    cv2.putText(
                        annotated, f"id={int(marker_id[0])}",
                        (text_pos[0], text_pos[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA
                    )

        return annotated, markers


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
