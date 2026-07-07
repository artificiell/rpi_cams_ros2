#!/usr/bin/env python3
"""
Serves camera images over HTTP as MJPEG streams, similar in spirit to
web_video_server, but sourced from ROS 2 services (cam_interfaces/srv/Image)
instead of topics.

Two service clients are used, both of the SAME service type:
  - camera/image  (always present)          -> raw stream at /stream
  - aruco/image   (present only when the    -> annotated stream at /stream_aruco
                    ArUco node's "display"
                    parameter is enabled)

Each stream is driven by a self-perpetuating async request chain (request ->
callback -> request again), so the capture rate is purely whatever the
underlying service can sustain, and any number of browser viewers just read
a shared cached JPEG rather than each triggering their own captures.

The ArUco stream's availability is polled periodically via the ROS graph
(service_is_ready). The index page at "/" shows one panel (raw only) or two
panels (raw + ArUco) depending on whether that service currently exists, and
auto-refreshes so this updates without restarting anything.

Parameters:
    camera_service_name  - default "camera/image"
    aruco_service_name   - default "aruco/image"
    jpeg_quality          - default 80
    http_port             - default 9090
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from cam_interfaces.srv import Image as ImageSrv


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_web_stream_bridge')

        self.declare_parameter('camera_service_name', 'camera/image')
        self.declare_parameter('aruco_service_name', 'aruco/image')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('http_port', 9090)

        camera_service_name = self.get_parameter('camera_service_name').value
        self.aruco_service_name = self.get_parameter('aruco_service_name').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.http_port = self.get_parameter('http_port').value

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_raw_jpeg = None
        self.latest_aruco_jpeg = None
        self.aruco_available = False

        # Client for the always-on raw camera service
        self.camera_client = self.create_client(ImageSrv, camera_service_name)
        self.get_logger().info(f"Waiting for '{camera_service_name}' service...")
        self.camera_client.wait_for_service()
        self.get_logger().info(f"'{camera_service_name}' service available.")

        # Client for the optional ArUco annotated-image service
        self.aruco_client = self.create_client(ImageSrv, self.aruco_service_name)

        # Start the always-on raw capture chain
        self.request_raw_capture()

        # Periodically check whether the ArUco service exists; (re)start its
        # request chain whenever it newly becomes available.
        self.create_timer(2.0, self.check_aruco_availability)

    # ---- Raw stream: self-perpetuating request chain, always running ----

    def request_raw_capture(self):
        future = self.camera_client.call_async(ImageSrv.Request())
        future.add_done_callback(self.handle_raw_response)

    def handle_raw_response(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                jpeg = self._encode_jpeg(response.image)
                if jpeg is not None:
                    with self.lock:
                        self.latest_raw_jpeg = jpeg
            else:
                self.get_logger().warn('Camera capture service reported failure.')
        except Exception as e:
            self.get_logger().error(f'Raw capture failed: {e}')

        # Keep the chain going regardless of outcome
        self.request_raw_capture()

    # ---- ArUco stream: only runs while the service is available ----

    def check_aruco_availability(self):
        available = self.aruco_client.service_is_ready()
        with self.lock:
            was_available = self.aruco_available
            self.aruco_available = available
            if not available:
                self.latest_aruco_jpeg = None  # drop stale frame once gone

        if available and not was_available:
            self.get_logger().info(f"'{self.aruco_service_name}' service detected, starting stream.")
            self.request_aruco_capture()
        elif not available and was_available:
            self.get_logger().info(f"'{self.aruco_service_name}' service no longer available.")

    def request_aruco_capture(self):
        future = self.aruco_client.call_async(ImageSrv.Request())
        future.add_done_callback(self.handle_aruco_response)

    def handle_aruco_response(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                jpeg = self._encode_jpeg(response.image)
                if jpeg is not None:
                    with self.lock:
                        self.latest_aruco_jpeg = jpeg
        except Exception as e:
            self.get_logger().warn(f'Aruco capture failed: {e}')

        # Only keep chaining while the service is still available; if it
        # disappeared, stop here. check_aruco_availability() will restart
        # the chain automatically once it comes back.
        with self.lock:
            still_available = self.aruco_available
        if still_available:
            self.request_aruco_capture()

    # ---- Shared helper ----

    def _encode_jpeg(self, img_msg):
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        ok, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return jpeg.tobytes() if ok else None


class MjpegHandler(BaseHTTPRequestHandler):
    node: CameraStreamNode = None  # injected before server starts

    def do_GET(self):
        if self.path == '/':
            self.serve_index()
        elif self.path == '/stream':
            self.serve_stream(kind='raw')
        elif self.path == '/aruco':
            self.serve_stream(kind='marker')
        else:
            self.send_response(404)
            self.end_headers()

    def serve_index(self):
        with self.node.lock:
            aruco_on = self.node.aruco_available

        panels = (
            '<div><h3 style="color:white;text-align:center;">Camera</h3>'
            '<img src="/stream" style="width:640px;"></div>'
        )
        if aruco_on:
            panels += (
                '<div><h3 style="color:white;text-align:center;">ArUco</h3>'
                '<img src="/aruco" style="width:640px;"></div>'
            )

        html = f"""
        <html>
        <head>
            <title>Robot Camera</title>
            <meta http-equiv="refresh" content="5">
        </head>
        <body style="background:#111; margin:0; display:flex; justify-content:center; gap:10px;">
            {panels}
        </body>
        </html>
        """
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def serve_stream(self, kind):
        if kind == 'marker':
            with self.node.lock:
                if not self.node.aruco_available:
                    self.send_response(503)
                    self.end_headers()
                    self.wfile.write(b'ArUco display service not available')
                    return

        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()

        try:
            while True:
                with self.node.lock:
                    frame = self.node.latest_aruco_jpeg if kind == 'marker' else self.node.latest_raw_jpeg
                if frame is not None:
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                time.sleep(1 / 15)  # cap the pace at which we re-check/send frames
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        return  # silence default per-request console logging


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    MjpegHandler.node = node
    server = ThreadingHTTPServer(('0.0.0.0', node.http_port), MjpegHandler)
    node.get_logger().info(f'Serving on http://0.0.0.0:{node.http_port}/')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
