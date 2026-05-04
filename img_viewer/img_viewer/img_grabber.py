#!/usr/bin/env python3
import os
from datetime import datetime
from zoneinfo import ZoneInfo

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from cam_interfaces.srv import Image


class ImgGrabber(Node):
    def __init__(self):
        super().__init__('img_grabber')

        self.declare_parameter('service_name', 'camera/image')

        self._service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self._tz = ZoneInfo('Europe/Stockholm')
        self._output_dir = os.path.expanduser('~/data/images/')
        self._bridge = CvBridge()

        os.makedirs(self._output_dir, exist_ok=True)

        self._client = self.create_client(Image, self._service_name)

        self.get_logger().info(f"Waiting for service '{self._service_name}' ...")
        while rclpy.ok() and not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service '{self._service_name}' not available yet...")
        self.get_logger().info(f"Connected to service: '{self._service_name}'")

        self._grab()

    # ---------------------------------------------------------------------------
    # Image request & response
    # ---------------------------------------------------------------------------

    def _grab(self):
        """Send a single image request to the service."""
        req = Image.Request()
        future = self._client.call_async(req)
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future):
        """Receive the image, save it to disk, then shut down."""
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Service returned no response.")
            else:
                img = self._bridge.imgmsg_to_cv2(response.image, desired_encoding='bgr8')
                filepath = self._build_filepath()
                cv2.imwrite(filepath, img)
                self.get_logger().info(f"Image saved to: {filepath}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        finally:
            rclpy.shutdown()

    # ---------------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------------

    def _build_filepath(self) -> str:
        """
        Construct the output path as:
          ~/data/images/<service_name_sanitized>_<YYYY-MM-DD_HH-MM-SS>.png
        The service name leading slash and any internal slashes are replaced
        with underscores so the result is a valid filename.
        """
        timestamp = datetime.now(self._tz).strftime('%Y-%m-%d_%H-%M-%S')
        sanitized = self._service_name.strip('/').replace('/', '_')
        filename = f"{sanitized}_{timestamp}.png"
        return os.path.join(self._output_dir, filename)

    def destroy_node(self):
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImgGrabber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
