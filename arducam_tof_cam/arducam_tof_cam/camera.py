#!/usr/bin/env python3
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cam_interfaces.srv import Image
from cv_bridge import CvBridge

import ArducamDepthCamera as ac


# Class for handle ArduCam Time-of-Flight (ToF) Camera
class ArduCamService(Node):
    def __init__(self):
        super().__init__('arducam_tof_camera_service')

        # Get ROS param
        self.declare_parameter('mode', 'depth')  # Can be "raw" or "depth"
        self.declare_parameter('framerate', 15)
        mode = self.get_parameter('mode').get_parameter_value().string_value.lower()
        self.frame_type = ac.FrameType.RAW if mode == 'raw' else ac.FrameType.DEPTH
        self.get_logger().info(f"Selected ArduCam ToF camera measure: {mode.upper()}")
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        
        # Inatailze the ArduCam camera
        self.cam = ac.ArducamCamera()
        self.initialized = self.init()

        # Setup ROS publisher or service
        self.bridge = CvBridge()
        self.service = self.create_service(Image, 'arducam/image', self.capture_callback)
        
    # Open and initialize the ArduCam camera
    def init(self) -> bool:
        self.get_logger().info(f"ArduCam SDK version: {ac.__version__}")
        ret = self.cam.open(ac.Connection.CSI)
        if ret != 0:
            self.get_logger().error(f"Failed to open ArduCam. Error code: {ret}")
            return False
        else:
            ret = self.cam.start(self.frame_type)
            if ret != 0:
                self.get_logger().error(f"Failed to start ArduCam. Error code: {ret}")
                self.cam.close()
                return False
            else:
                if self.frame_type == ac.FrameType.DEPTH:
                    self.cam.setControl(ac.Control.RANGE, 4000)
                    self.range = self.cam.getControl(ac.Control.RANGE)
                self.get_logger().info("ArduCam initialized successfully.")
                info = self.cam.getCameraInfo()
                self.get_logger().info(f"Camera resolution: {info.width}x{info.height} @ {self.framerate} frames / second")
                self.get_logger().info(f"Camera type: {info.device_type}")
                if self.frame_type == ac.FrameType.DEPTH:
                    self.get_logger().info(f"Camera depth range: {self.range / 1000.0} m")
                return True

    # Cature frame from the ArduCam camera
    def capture_callback(self, request: Image.Request, response: Image.Response) -> None:
        response.success = False
        if self.initialized:
            frame = self.cam.requestFrame(200)
            if frame is not None:
                try: 
                    if self.frame_type == ac.FrameType.RAW and isinstance(frame, ac.RawData):
                        buf = frame.raw_data
                        buf = (buf / (1 << 4)).astype(np.uint8)
                        response.image = self.bridge.cv2_to_imgmsg(buf, encoding = 'mono8')    
                    elif self.frame_type == ac.FrameType.DEPTH and isinstance(frame, ac.DepthData):
                        depth_buf = frame.depth_data
                        confidence_buf = frame.confidence_data
                        
                        # Scale and colorize
                        scaled = (depth_buf * (255.0 / self.range)).astype(np.uint8)
                        colored = cv2.applyColorMap(scaled, cv2.COLORMAP_RAINBOW)
                        colored[confidence_buf < 30] = (0, 0, 0)  # Confidence filter
                        response.image = self.bridge.cv2_to_imgmsg(colored, encoding='bgr8')
                    response.success = True
                except Exception as e:
                    self.get_logger().error(f'Camera capture error: {e}')
                finally:
                    self.cam.releaseFrame(frame)
                    
    # Clean up
    def destroy_node(self) -> None:
        if hasattr(self, 'cam'):
            self.cam.stop()
            self.cam.close()
        super().destroy_node()
        

# Main fn
def main(args=None):
    rclpy.init(args=args)
    node = ArduCamService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()

