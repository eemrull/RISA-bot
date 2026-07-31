#!/usr/bin/env python3
"""
ros2go2rtc_bridge — Camera streaming bridge for go2rtc (Pull Architecture)
=============================================================================
Subscribes to ROS camera view topics and serves a clean HTTP MJPEG stream on
port 1985. go2rtc pulls on-demand from http://127.0.0.1:1985/mjpeg.

When no viewers are connected to go2rtc, go2rtc disconnects from port 1985,
client count drops to 0, and JPEG encoding is completely paused to save CPU.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import Dict, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image

from .topics import (
    CAMERA_DEBUG_LINE_TOPIC,
    CAMERA_DEBUG_OBS_TOPIC,
    CAMERA_IMAGE_TOPIC,
    SIGNAGE_DEBUG_TOPIC,
)

# Dashboard view name -> source topic. Names must match the values the dashboard
# sends to /api/set_cam_view. 'traffic_light' and 'signage' deliberately share a
# topic: traffic_light_detector is not launched, so /camera/debug/traffic_light is
# never published — signage_detector handles traffic lights via its YOLO classes.
VIEW_TOPICS = {
    'raw':           CAMERA_IMAGE_TOPIC,
    'line_follower': CAMERA_DEBUG_LINE_TOPIC,
    'obstacle':      CAMERA_DEBUG_OBS_TOPIC,
    'signage':       SIGNAGE_DEBUG_TOPIC,
    'traffic_light': SIGNAGE_DEBUG_TOPIC,
}

HTTP_PORT = 1985


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Multi-threaded HTTP server for MJPEG clients."""
    daemon_threads = True
    allow_reuse_address = True


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """HTTP Handler serving an MJPEG stream to go2rtc."""

    node_ref: Optional['Go2rtcBridgeNode'] = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if not self.node_ref:
            self.send_error(500, "Node reference missing")
            return

        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Connection', 'close')
        self.end_headers()

        self.node_ref.register_client()
        try:
            while True:
                frame_data = self.node_ref.get_next_frame(timeout=1.0)
                if frame_data:
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(frame_data)).encode() + b'\r\n\r\n'
                        + frame_data + b'\r\n'
                    )
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.node_ref.unregister_client()


class Go2rtcBridgeNode(Node):
    """ROS 2 Bridge Node hosting MJPEG server for go2rtc."""

    def __init__(self) -> None:
        super().__init__('ros2go2rtc_bridge')

        self.declare_parameter('active_view', 'raw')
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('http_port', HTTP_PORT)

        # Fall back to 'raw' rather than trusting the launch override — an unknown
        # name here would KeyError on every frame in _image_cb.
        initial_view = str(self.get_parameter('active_view').value)
        if initial_view not in VIEW_TOPICS:
            self.get_logger().warn(
                f'Unknown active_view {initial_view!r} at startup; falling back to "raw". '
                f'Valid: {list(VIEW_TOPICS.keys())}'
            )
            initial_view = 'raw'

        self._param_cache = {
            'active_view': initial_view,
            'jpeg_quality': int(self.get_parameter('jpeg_quality').value),
            'resize_width': int(self.get_parameter('resize_width').value),
            'http_port': int(self.get_parameter('http_port').value),
        }
        self.add_on_set_parameters_callback(self._on_params)

        self.bridge = CvBridge()

        self._client_count = 0
        self._lock = threading.Lock()
        self._frame_cond = threading.Condition(self._lock)
        self._latest_jpeg: Optional[bytes] = None

        # Start HTTP server on port 1985
        MJPEGStreamHandler.node_ref = self
        port = self._param_cache['http_port']
        self._httpd = ThreadedHTTPServer(('0.0.0.0', port), MJPEGStreamHandler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        # Subscribe once per distinct topic — several views can share one
        # (e.g. 'signage' and 'traffic_light'), and subscribing twice would
        # double the callback work for no benefit.
        self._subs: Dict[str, rclpy.subscription.Subscription] = {}
        # SENSOR_DATA (BEST_EFFORT) is required: the Astra driver publishes
        # /camera/color/image_raw with sensor QoS, and a RELIABLE subscriber is
        # incompatible with it — the subscription binds but receives nothing.
        # BEST_EFFORT also accepts the RELIABLE debug-image publishers, so one
        # profile covers every view.
        for topic in sorted(set(VIEW_TOPICS.values())):
            self._subs[topic] = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self._image_cb(msg, t),
                QoSPresetProfiles.SENSOR_DATA.value,
            )

        self.get_logger().info(
            f'ros2go2rtc_bridge online | serving MJPEG on http://0.0.0.0:{port}/mjpeg | view: {self._param_cache["active_view"]}'
        )

    def register_client(self) -> None:
        with self._lock:
            self._client_count += 1
            self.get_logger().info(f'go2rtc connected to bridge (active viewers: {self._client_count})')

    def unregister_client(self) -> None:
        with self._lock:
            self._client_count = max(0, self._client_count - 1)
            self.get_logger().info(f'go2rtc disconnected from bridge (active viewers: {self._client_count})')

    def get_next_frame(self, timeout: float = 1.0) -> Optional[bytes]:
        with self._frame_cond:
            if self._frame_cond.wait(timeout=timeout):
                return self._latest_jpeg
            return None

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'active_view':
                if p.value in VIEW_TOPICS:
                    self._param_cache['active_view'] = str(p.value)
                    self.get_logger().info(f'Camera view switched to: {p.value}')
                else:
                    self.get_logger().warn(f'Unknown view: {p.value!r}. Valid: {list(VIEW_TOPICS.keys())}')
                    return SetParametersResult(successful=False, reason=f'Unknown view "{p.value}"')
            elif p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _image_cb(self, msg: Image, topic: str) -> None:
        if topic != VIEW_TOPICS[self._param_cache['active_view']]:
            return

        # Pause encoding completely when no clients are connected to port 1985
        with self._lock:
            if self._client_count == 0:
                return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            w = self._param_cache['resize_width']
            if w > 0:
                ih, iw = cv_image.shape[:2]
                if iw != w:
                    h = int(ih * w / iw)
                    cv_image = cv2.resize(cv_image, (w, h))

            ok, jpeg_buf = cv2.imencode(
                '.jpg', cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, self._param_cache['jpeg_quality']]
            )
            if not ok:
                return

            with self._frame_cond:
                self._latest_jpeg = jpeg_buf.tobytes()
                self._frame_cond.notify_all()

        except Exception as e:
            self.get_logger().error(f'Frame processing error: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2rtcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._httpd.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
