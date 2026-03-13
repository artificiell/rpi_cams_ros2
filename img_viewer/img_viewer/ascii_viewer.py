#!/usr/bin/env python3
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from cam_interfaces.srv import Image


class AsciiImageViewer(Node):
    def __init__(self):
        super().__init__('ascii_image_viewer')

        # Declare and get parameters
        self.declare_parameter('screen_width', 80)
        self.declare_parameter('service_name', 'camera/capture_image')
        self.width = self.get_parameter('screen_width').get_parameter_value().integer_value
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        # Detect truecolor support
        colorterm = os.environ.get("COLORTERM", "")
        self.truecolor = colorterm in ["truecolor", "24bit"]
        self.get_logger().info(f"Terminal truecolor support: {self.truecolor}")
        
        # Set up ROS service client
        self.bridge = CvBridge()
        self.client = self.create_client(Image, self.service_name)

        # Wait until the camera service exists before starting
        self.get_logger().info(f"Waiting for service '{self.service_name}' ...")
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info(f"Service '{self.service_name}' not available yet...")
        self.get_logger().info(f"Connected to service: '{self.service_name}'")
        self.pending_future = None
        self.stopped = False
        self.request_image()  # ...send request for first image

    # Send request for image asynchronously
    def request_image(self):
        if self.stopped or not rclpy.ok():
            return

        if self.pending_future is not None and not self.pending_future.done():
            return

        req = Image.Request()
        self.pending_future = self.client.call_async(req)
        self.pending_future.add_done_callback(self.handle_response)
        
    '''
    # Callback for receiving and displaying images
    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            os.system('clear')
            if self.truecolor:
                print(self.image_to_color_ascii(img, width = self.width))
            else:
                print(self.image_to_grayscale_ascii(img, width = self.width))
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    '''
    
    # Async handling of service response 
    def handle_response(self, future):
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Service returned no response.")
            else:
                msg = response.image
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
                print("\x1b[2J\x1b[H", end='') # ...clear screen
                if self.truecolor:
                    print(self.image_to_color_ascii(img, width = self.width), flush = True)
                else:
                    print(self.image_to_grayscale_ascii(img, width = self.width), flush = True)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        # Immediately requesting the next frame
        if rclpy.ok() and not self.stopped:
            self.request_image()
            
    # Convert image to color ASCII 
    def image_to_color_ascii(self, img, width):
        block = '▀' # Unicode upper-half block
        h, w, _ = img.shape
        if w == 0 or h == 0:
            return "[empty image]"

        aspect_ratio = h / float(w)
        new_height = max(1, int(aspect_ratio * width * 0.5))
        resized = cv2.resize(img, (width, new_height * 2))

        output_lines = []
        for y in range(0, resized.shape[0] - 1, 2):
            top = resized[y]
            bottom = resized[y + 1]
            line = ''
            for t, b in zip(top, bottom):
                r1, g1, b1 = t
                r2, g2, b2 = b
                line += f"\x1b[38;2;{r1};{g1};{b1}m\x1b[48;2;{r2};{g2};{b2}m{block}"
            line += "\x1b[0m"
            output_lines.append(line)

        return "\n".join(output_lines)

    # Convert image to grayscael ASCII 
    def image_to_grayscale_ascii(self, img, width):
        ascii_chars = np.array(list(" .:-=+*#%@"))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        if w == 0 or h == 0:
            return "[empty image]"

        aspect_ratio = h / float(w)
        new_height = max(1, int(aspect_ratio * width * 0.5))
        resized = cv2.resize(gray, (width, new_height))

        # Normalize and convert to indices
        scaled = (resized / 255.0) * (len(ascii_chars) - 1)
        indices = scaled.astype(np.uint8)
        ascii_img = np.take(ascii_chars, indices)

        return "\n".join("".join(row) for row in ascii_img)

    # Clean up
    def destroy_node(self):
        self.stopped = True
        super().destroy_node()
    
    
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AsciiImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
