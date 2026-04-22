#!/usr/bin/env python3
import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge

from cam_interfaces.srv import Image

class StreamImageViewer(Node):
    def __init__(self):
        super().__init__('opencv_image_viewer')

        # Parameters
        self.declare_parameter('service_name', 'camera/capture_image')
        self.declare_parameter('window_name', 'Camera Frame')

        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.client = self.create_client(Image, self.service_name)

        self.get_logger().info(f"Waiting for service '{self.service_name}' ...")
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service '{self.service_name}' not available yet...")
        self.get_logger().info(f"Connected to service: '{self.service_name}'")

        self.pending_future = None
        self.stopped = False

        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # Request first frame
        self.request_image()

    def request_image(self):
        if self.stopped or not rclpy.ok():
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        req = Image.Request()
        self.pending_future = self.client.call_async(req)
        self.pending_future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Service returned no response.")
            else:
                msg = response.image
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                cv2.imshow(self.window_name, img)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q') or key == 27:
                    self.get_logger().info("Quit key pressed, shutting down.")
                    self.stopped = True
                    rclpy.shutdown()
                    return

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        if rclpy.ok() and not self.stopped:
            self.request_image()

    def destroy_node(self):
        self.stopped = True
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StreamImageViewer()

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
