#!/usr/bin/env python3
"""
Robot Dashboard — Web-based status monitor
Subscribes to all key ROS topics and serves a live web dashboard
at http://<robot_ip>:8080

Run standalone:  python3 dashboard.py
Or via launch:   included in competition.launch.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Image
from nav_msgs.msg import Odometry
from rclpy.qos import QoSPresetProfiles
import threading
import json
import time
import http.server
import socketserver
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as RosParameter, ParameterValue, ParameterType
import math
import numpy as np
import cv2

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

# ======================== HTML Dashboard ========================

from .dashboard_templates import DASHBOARD_HTML, TEACH_HTML

# ======================== ROS2 Dashboard Node ========================

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.get_logger().info('Dashboard node starting on http://0.0.0.0:8080')

        # CV Bridge for camera
        self.bridge = CvBridge() if CvBridge else None
        self.latest_jpeg = None
        self.jpeg_lock = threading.Lock()
        self.active_camera_view = 'raw'
        self.initial_joy_axes = None

        # Shared state (read by HTTP handler)
        self.data = {
            'state': 'LANE_FOLLOW',
            'lap': 1,
            'auto_mode': False,
            'state_time': 0,
            'state_dist': '0.00',
            'traffic_light': 'unknown',
            'lidar_obstacle': None,
            'camera_obstacle': None,
            'fused_obstacle': None,
            'boom_gate': None,
            'tunnel_detected': None,
            'obstruction_active': None,
            'parking_complete': None,
            'stop_reason': '',
            'lane_error': 0.0,
            'cmd_lin_x': 0.0,
            'cmd_ang_z': 0.0,
            'distance': 0.0,
            'speed': 0.0,
            'odom_x': 0.0,
            'odom_y': 0.0,
            'odom_yaw': 0.0,
            'buttons': [],
            'axes': [],
            'speed_pct': 25,
            'ctrl_state_index': 0,
            'ctrl_state_name': 'LANE_FOLLOW',
        }
        self.data_lock = threading.Lock()

        # State tracking
        self._state_entry_time = time.time()

        # === Subscriptions ===
        qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(Bool, '/auto_mode', self._auto_mode_cb, 10)
        self.create_subscription(Bool, '/obstacle_front', self._lidar_cb, qos)
        self.create_subscription(Bool, '/obstacle_detected_camera', self._cam_cb, qos)
        self.create_subscription(Bool, '/obstacle_detected_fused', self._fused_cb, 10)
        self.create_subscription(String, '/traffic_light_state', self._tl_cb, 10)
        self.create_subscription(Bool, '/boom_gate_open', self._gate_cb, 10)
        self.create_subscription(Bool, '/tunnel_detected', self._tunnel_cb, 10)
        self.create_subscription(Bool, '/obstruction_active', self._obst_cb, 10)
        self.create_subscription(Bool, '/parking_complete', self._park_cb, 10)
        self.create_subscription(Float32, '/lane_error', self._lane_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, '/dashboard_state', self._dash_state_cb, 10)
        self.create_subscription(String, '/dashboard_ctrl', self._dash_ctrl_cb, 10)
        self.create_subscription(String, '/set_challenge', self._set_challenge_cb, 10)
        # Also listen to auto commands for display
        self.create_subscription(Twist, '/cmd_vel_auto', self._cmd_cb, 10)

        # Camera subscriptions (SENSOR_DATA QoS to match camera publisher)
        self.create_subscription(Image, '/camera/color/image_raw', lambda msg: self._image_cb(msg, 'raw'), qos)
        self.create_subscription(Image, '/camera/debug/line_follower', lambda msg: self._image_cb(msg, 'line_follower'), qos)
        self.create_subscription(Image, '/camera/debug/traffic_light', lambda msg: self._image_cb(msg, 'traffic_light'), qos)
        self.create_subscription(Image, '/camera/debug/obstacle', lambda msg: self._image_cb(msg, 'obstacle'), qos)

        # Simulate odometry since hardware might not publish
        self.create_timer(0.05, self._simulate_odom_loop)

        self.get_logger().info('Dashboard subscriptions ready')

    def _simulate_odom_loop(self):
        """Simulates odometry position based on commanded velocities.
        Useful when the hardware driver fails to publish /odom data."""
        now = time.time()
        if not hasattr(self, '_last_sim_t'):
            self._last_sim_t = now
            return
            
        dt = min(now - self._last_sim_t, 0.2)
        self._last_sim_t = now
        
        with self.data_lock:
            # If no cmd_vel received in last 0.5s, assume robot is stopped
            cmd_age = now - self.data.get('_cmd_time', 0)
            if cmd_age > 0.5:
                vel_x = 0.0
                vel_z = 0.0
            else:
                vel_x = self.data['cmd_lin_x']
                vel_z = self.data['cmd_ang_z']
            
            # Odometry calibration: measured 1m real ≈ 0.87m at 1.54x → refined to 1.77x
            # Residual error is from wheel slip/coasting (robot moves after cmd_vel=0)
            odom_scale = 1.67
            cal_vel_x = vel_x * odom_scale
            
            # Distance integration
            self.data['speed'] = cal_vel_x
            self.data['distance'] += abs(cal_vel_x) * dt
            
            # Position integration (dead reckoning)
            self.data['odom_yaw'] += vel_z * dt
            
            # X and Y based on current heading
            self.data['odom_x'] += cal_vel_x * math.cos(self.data['odom_yaw']) * dt
            self.data['odom_y'] += cal_vel_x * math.sin(self.data['odom_yaw']) * dt

    def _set(self, key, value):
        with self.data_lock:
            self.data[key] = value

    def _auto_mode_cb(self, msg):
        self._set('auto_mode', msg.data)
        mode = "AUTO" if msg.data else "MANUAL"
        self.get_logger().info(f'Mode changed: {mode}')

    def _lidar_cb(self, msg): self._set('lidar_obstacle', msg.data)
    def _cam_cb(self, msg): self._set('camera_obstacle', msg.data)
    def _fused_cb(self, msg): self._set('fused_obstacle', msg.data)
    def _tl_cb(self, msg): self._set('traffic_light', msg.data)
    def _gate_cb(self, msg): self._set('boom_gate', msg.data)
    def _tunnel_cb(self, msg): self._set('tunnel_detected', msg.data)
    def _obst_cb(self, msg): self._set('obstruction_active', msg.data)
    def _park_cb(self, msg): self._set('parking_complete', msg.data)
    def _lane_cb(self, msg): self._set('lane_error', msg.data)

    def _cmd_cb(self, msg):
        with self.data_lock:
            self.data['cmd_lin_x'] = msg.linear.x
            self.data['cmd_ang_z'] = msg.angular.z
            self.data['_cmd_time'] = time.time()

    def _odom_cb(self, msg):
        # We receive actual odometry! Use real data instead of dead reckoning.
        with self.data_lock:
            self.data['speed'] = msg.twist.twist.linear.x
            now = time.time()
            if hasattr(self, '_last_real_odom_t'):
                dt = min(now - self._last_real_odom_t, 0.2)
                self.data['distance'] += abs(msg.twist.twist.linear.x) * dt
            self._last_real_odom_t = now

            # Raw Pose
            self.data['odom_x'] = msg.pose.pose.position.x
            self.data['odom_y'] = msg.pose.pose.position.y
            
            # Quaternion to Yaw (Euler Z)
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.data['odom_yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def _joy_cb(self, msg):
        with self.data_lock:
            self.data['buttons'] = list(msg.buttons)
            if self.initial_joy_axes is None:
                self.initial_joy_axes = list(msg.axes)
            
            if self.initial_joy_axes and list(msg.axes) == self.initial_joy_axes:
                self.data['axes'] = [0.0] * len(msg.axes)
            else:
                self.initial_joy_axes = []  # Clear forever once moved
                self.data['axes'] = list(msg.axes)

    def _dash_state_cb(self, msg):
        """Parse STATE|LAP|TOTAL_DIST|STOP_REASON format from auto_driver."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                if parts[0] != self.data.get('state'):
                    self._state_entry_time = time.time()  # Reset timer on state change
                self.data['state'] = parts[0]
                if len(parts) > 1:
                    self.data['lap'] = int(parts[1])
                if len(parts) > 2:
                    self.data['state_dist'] = parts[2]  # Now represents total distance
                if len(parts) > 3:
                    self.data['stop_reason'] = parts[3]
                else:
                    self.data['stop_reason'] = ''
        except Exception:
            self._set('state', msg.data)

    def _dash_ctrl_cb(self, msg):
        """Parse SPD_PCT|STATE_INDEX|STATE_NAME format from servo_controller."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                self.data['speed_pct'] = int(parts[0])
                if len(parts) > 1:
                    self.data['ctrl_state_index'] = int(parts[1])
                if len(parts) > 2:
                    self.data['ctrl_state_name'] = parts[2]
        except Exception:
            pass

    def _set_challenge_cb(self, msg):
        """Also listen to /set_challenge as fallback for state cycle display."""
        name = msg.data.upper()
        self._set('ctrl_state_name', name)

    def _image_cb(self, msg, view_name):
        """Convert ROS Image to JPEG for the web feed, tracking which view is active."""
        if self.bridge is None or view_name != self.active_camera_view:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize to save bandwidth
            h, w = cv_image.shape[:2]
            scale = min(640 / w, 480 / h)
            if scale < 1.0:
                cv_image = cv2.resize(cv_image, (int(w * scale), int(h * scale)))
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with self.jpeg_lock:
                self.latest_jpeg = jpeg.tobytes()
        except Exception:
            pass

    def get_json(self):
        with self.data_lock:
            d = dict(self.data)
        d['state_time'] = int(time.time() - self._state_entry_time)
        return json.dumps(d)

    def get_jpeg(self):
        with self.jpeg_lock:
            return self.latest_jpeg


# ======================== HTTP Server ========================

# Cached service clients for parameter operations
_param_clients_lock = threading.Lock()
_param_clients = {}  # {'/node_name': {'get': client, 'set': client}}

def _get_client(node_name, svc_type):
    """Get or create a cached service client."""
    n = node_name if node_name.startswith('/') else '/' + node_name
    key = (n, svc_type)
    with _param_clients_lock:
        if key not in _param_clients:
            if svc_type == 'get':
                _param_clients[key] = _node_ref.create_client(GetParameters, n + '/get_parameters')
            else:
                _param_clients[key] = _node_ref.create_client(SetParameters, n + '/set_parameters')
    return _param_clients[key]

def _ros_get_param(node_name, param_name):
    """Get a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'get')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return None, f'Node /{node_name} not available'
        req = GetParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None, 'Timeout'
        res = future.result()
        if res and res.values:
            v = res.values[0]
            if v.type == ParameterType.PARAMETER_BOOL:
                return str(v.bool_value).lower(), None
            elif v.type == ParameterType.PARAMETER_INTEGER:
                return str(v.integer_value), None
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                return str(v.double_value), None
            elif v.type == ParameterType.PARAMETER_STRING:
                return v.string_value, None
            elif v.type == ParameterType.PARAMETER_NOT_SET:
                return None, 'Not set'
            else:
                return '?', None
        return None, 'No result'
    except Exception as e:
        return None, str(e)

def _ros_set_param(node_name, param_name, value_str):
    """Set a parameter via native rclpy service (no subprocess)."""
    try:
        client = _get_client(node_name, 'set')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return False, f'Node /{node_name} not available'
        param = RosParameter()
        param.name = param_name
        pv = ParameterValue()
        # Infer type from string
        if value_str.lower() in ('true', 'false'):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value_str.lower() == 'true'
        else:
            try:
                # Check if it's a pure integer (no decimal point)
                if '.' not in value_str:
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(value_str)
                else:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(value_str)
            except ValueError:
                pv.type = ParameterType.PARAMETER_STRING
                pv.string_value = value_str
        param.value = pv
        req = SetParameters.Request()
        req.parameters = [param]
        future = client.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, 'Timeout'
        res = future.result()
        if res and res.results:
            r = res.results[0]
            if r.successful:
                return True, 'OK'
            else:
                return False, r.reason or 'Failed'
        return False, 'No result'
    except Exception as e:
        return False, str(e)

_node_ref = None

class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/data':
            data = _node_ref.get_json() if _node_ref else '{}'
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data.encode())
        elif self.path.startswith('/camera_feed'):
            jpeg = _node_ref.get_jpeg() if _node_ref else None
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        jpeg1 = _node_ref.get_jpeg()
                        if jpeg1:
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(jpeg1)))
                            self.end_headers()
                            self.wfile.write(jpeg1)
                            self.wfile.write(b'\r\n')
                        time.sleep(0.05)
                except Exception:
                    pass
            else:
                self.send_error(404)
        elif self.path.startswith('/api/set_cam_view'):
            from urllib.parse import urlparse, parse_qs
            import threading
            qs = parse_qs(urlparse(self.path).query)
            view = qs.get('view', ['raw'])[0]
            if _node_ref:
                _node_ref.active_camera_view = view
            
            # Auto-toggle show_debug for performance 
            def auto_toggle_debug(selected_view):
                mapping = {
                    'line_follower': 'line_follower_camera',
                    'traffic_light': 'traffic_light_detector',
                    'obstacle': 'obstacle_avoidance_camera'
                }
                for v, node_name in mapping.items():
                    val_str = 'true' if v == selected_view else 'false'
                    _ros_set_param(node_name, 'show_debug', val_str)
                    
            threading.Thread(target=auto_toggle_debug, args=(view,), daemon=True).start()

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        elif self.path.startswith('/api/get_param'):
            from urllib.parse import urlparse, parse_qs
            qs = parse_qs(urlparse(self.path).query)
            node = qs.get('node', [''])[0]
            param = qs.get('param', [''])[0]
            result = {'ok': False}
            if node and param:
                value, err = _ros_get_param(node, param)
                if err is None:
                    result = {'ok': True, 'value': value}
                else:
                    result = {'ok': False, 'error': err}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        elif self.path.startswith('/teach'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(TEACH_HTML.encode())
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

    def do_POST(self):
        if self.path == '/api/reset_odom':
            # Reset odometry counters
            global _node_ref
            if _node_ref:
                with _node_ref.data_lock:
                    _node_ref.data['distance'] = 0.0
                    _node_ref.data['odom_x'] = 0.0
                    _node_ref.data['odom_y'] = 0.0
                    _node_ref.data['odom_yaw'] = 0.0
                    _node_ref.data['speed'] = 0.0
            resp = {'ok': True, 'msg': 'Odometry reset'}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        elif self.path == '/api/set_param':
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len)
            try:
                data = json.loads(body)
                node = data['node']
                param = data['param']
                value = str(data['value'])
                ok, msg = _ros_set_param(node, param, value)
                if ok:
                    resp = {'ok': True, 'msg': msg}
                else:
                    resp = {'ok': False, 'error': msg}
            except Exception as e:
                resp = {'ok': False, 'error': str(e)}
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(resp).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


def main(args=None):
    global _node_ref
    rclpy.init(args=args)
    node = DashboardNode()
    _node_ref = node

    # Start HTTP server in background thread
    class ThreadedHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
    server = ThreadedHTTPServer(('0.0.0.0', 8080), DashboardHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    # Print user-friendly access URLs
    import socket
    hostname = socket.gethostname()
    try:
        ip = socket.getsockname() if hasattr(socket, 'getsockname') else '?'
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = '?.?.?.?'
    node.get_logger().info(f'Dashboard live!')
    node.get_logger().info(f'  → http://{hostname}.local:8080')
    node.get_logger().info(f'  → http://{ip}:8080')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
