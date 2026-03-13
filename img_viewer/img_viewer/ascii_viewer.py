#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class AsciiImageViewer(Node):
    def __init__(self):
        super().__init__('ascii_image_viewer')

        # Declare and get parameters
        self.declare_parameter('width', 80)
        self.width = self.get_parameter('width').get_parameter_value().integer_value

        # Detect truecolor support
        colorterm = os.environ.get("COLORTERM", "")
        self.truecolor = colorterm in ["truecolor", "24bit"]
        self.get_logger().info(f"Terminal truecolor support: {self.truecolor}")
        
        # Set up ROS subscriber
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            qos_profile_sensor_data
        )

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
    
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AsciiImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
