#!/usr/bin/env python3
"""
This node continuously calls the RPi camera capture service
(cam_interfaces/srv/Image) to obtain frames, as fast as that service can
sustain (no artificial rate limit), locates ArUco AR markers in each one,
and:

  1. Publishes their ids and poses on a topic, at that same natural rate.
     This is small structured data, so it doesn't run into the bandwidth/
     stability issues that motivated moving images to services, and it is
     ALWAYS published regardless of the "display" setting below.
  2. Optionally serves the most recently annotated frame (markers + axes
     drawn on top of the raw frame) via its own service, reusing the same
     cam_interfaces/srv/Image type as the camera capture service. This
     service is only created while the "display" parameter is true, and
     can be toggled on/off at runtime. It never triggers a new capture —
     it just hands back whatever the detection loop most recently produced.

Service client:
    <service_name> (cam_interfaces.srv.Image), default "camera/image"

Service server (only while display == True):
    <aruco_service_name> (cam_interfaces.srv.Image), default "aruco/image"
       Returns the most recently detected+annotated frame.

Published Topics:
    aruco/markers (cam_interfaces.msg.ArucoMarkers)
       Array of all detected marker ids and poses, relative to the camera.
       Published continuously, independent of the display service.

Parameters:
    marker_size          - size of the markers in meters (default 0.1)
    aruco_dictionary_id  - dictionary used to generate markers
                            (default DICT_ARUCO_ORIGINAL)
    service_name         - camera capture service to call (default "camera/image")
    aruco_service_name   - name of this node's own annotated-image service
                            (default "aruco/image")
    camera_frame         - camera optical frame to use (default: taken from the
                            capture service's response info if left blank)
    display               - whether to expose the aruco_service_name service
                            (default True). Can be toggled at runtime via
                            `ros2 param set <node> display <true|false>`.
"""

import rclpy
from rclpy.node import Node
from packaging.version import Version
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from geometry_msgs.msg import Pose

from cam_interfaces.srv import Image
from cam_interfaces.msg import ArucoMarker, ArucoMarkers


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
            name="camera_service_name",
            value="camera/image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera capture service to call (cam_interfaces/srv/Image).",
            ),
        )
        self.declare_parameter(
            name="aruco_service_name",
            value="aruco/image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Service name for the annotated ArUco image (cam_interfaces/srv/Image).",
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
        self.declare_parameter(
            name="display",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to expose the annotated-image service (aruco_service_name).",
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

        capture_service_name = (
            self.get_parameter("camera_service_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera capture service: {capture_service_name}")

        self.aruco_service_name = (
            self.get_parameter("aruco_service_name").get_parameter_value().string_value
        )

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        self.display = self.get_parameter("display").get_parameter_value().bool_value
        self.get_logger().info(f"Display (aruco/image service) enabled: {self.display}")

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

        # Client: calls the existing camera capture service
        self.capture_client = self.create_client(Image, capture_service_name)
        while not self.capture_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Waiting for capture service '{capture_service_name}'...")

        # Set up publisher for marker ids/poses (always on, regardless of display)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco/markers", 10)

        # Cache of the most recent detection result. The display service (if
        # enabled) just hands this back — it never triggers a capture itself.
        self.latest_annotated_image = None
        self.latest_info = None
        self.latest_success = False

        # Server: exposes the annotated-image service, only while display == True
        self.aruco_service = None
        self._update_display_service(self.display)

        # Allow "display" to be toggled at runtime, e.g.:
        #   ros2 param set /aruco_detection_node display false
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # Kick off the request chain: as soon as one captured image has been
        # processed, the next one is requested immediately. This is what
        # drives the detection/publish rate — purely however fast the
        # capture service can sustain, with no separate rate parameter.
        self.request_image_capture()

    def _update_display_service(self, enable: bool):
        """Create or destroy the aruco/image service based on the display flag."""
        if enable and self.aruco_service is None:
            self.aruco_service = self.create_service(
                Image, self.aruco_service_name, self.handle_aruco_request
            )
            self.get_logger().info(f"Display enabled: '{self.aruco_service_name}' service created.")
        elif not enable and self.aruco_service is not None:
            self.destroy_service(self.aruco_service)
            self.aruco_service = None
            self.get_logger().info(f"Display disabled: '{self.aruco_service_name}' service destroyed.")

    def on_parameter_change(self, params):
        for param in params:
            if param.name == "display":
                self.display = param.value
                self._update_display_service(self.display)
        return SetParametersResult(successful=True)

    def request_image_capture(self):
        """Fire off a capture request; the next one is requested as soon as
        this one's response has been handled (see handle_image_response)."""
        request = Image.Request()
        future = self.capture_client.call_async(request)
        future.add_done_callback(self.handle_image_response)

    def handle_image_response(self, future):
        try:
            capture_response = future.result()
            if capture_response is None or not capture_response.success:
                self.get_logger().warn("Capture service reported failure, skipping frame.")
                self.latest_success = False
            else:
                try:
                    self.process_and_publish(capture_response.image, capture_response.info)
                except Exception as e:
                    self.get_logger().error(f"Error processing frame: {e}")
                    self.latest_success = False
        except Exception as e:
            self.get_logger().error(f"Capture service call failed: {e}")
            self.latest_success = False

        # Immediately request the next frame — no fixed rate, just whatever
        # the capture service (and the processing below) can sustain.
        self.request_image_capture()

    def process_and_publish(self, img_msg, info_msg):
        """Detect markers, draw them, cache the annotated frame, and publish poses."""
        annotated, markers = self.process_frame(img_msg, info_msg)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = img_msg.header

        self.latest_annotated_image = annotated_msg
        self.latest_info = info_msg
        self.latest_success = True

        markers_msg = ArucoMarkers()
        markers_msg.header = annotated_msg.header
        if self.camera_frame:
            markers_msg.header.frame_id = self.camera_frame
        markers_msg.markers = markers
        self.markers_pub.publish(markers_msg)

    def handle_aruco_request(self, request, response):
        """Just hands back the most recently detected+annotated frame.
        Does NOT trigger a new capture — that loop runs independently."""
        response.success = self.latest_success
        if self.latest_success:
            response.image = self.latest_annotated_image
            response.info = self.latest_info
        return response

    def process_frame(self, img_msg, info_msg):
        intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        distortion = np.array(info_msg.d)

        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        annotated = image.copy()
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

            cv2.aruco.drawDetectedMarkers(annotated, corners, marker_ids)

            for i, marker_id in enumerate(marker_ids):
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
