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
import numpy as np
import cv2

# ======================== HTML Dashboard ========================

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap" rel="stylesheet">
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: #0f0f1a;
    color: #e0e0e0;
    min-height: 100vh;
  }
  .header {
    background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
    padding: 16px 24px;
    border-bottom: 2px solid #0f3460;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .header h1 {
    font-size: 1.4em;
    font-weight: 700;
    background: linear-gradient(135deg, #e94560, #0f3460);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
  }
  .header .status-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: #4caf50;
    display: inline-block;
    animation: pulse 2s infinite;
    margin-right: 8px;
  }
  @keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.4; }
  }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 12px;
    padding: 16px;
    max-width: 1200px;
    margin: 0 auto;
  }
  .card {
    background: linear-gradient(145deg, #1a1a2e, #16213e);
    border: 1px solid #0f346060;
    border-radius: 12px;
    padding: 16px;
  }
  .card h3 {
    font-size: 0.75em;
    text-transform: uppercase;
    letter-spacing: 1.5px;
    color: #888;
    margin-bottom: 10px;
  }
  .card.wide { grid-column: span 2; }
  .card.full { grid-column: span 3; }

  /* State badge */
  .state-badge {
    display: inline-block;
    padding: 8px 18px;
    border-radius: 8px;
    font-size: 1.3em;
    font-weight: 700;
    letter-spacing: 1px;
  }
  .state-LANE_FOLLOW { background: #1b5e20; color: #a5d6a7; }
  .state-OBSTRUCTION { background: #e65100; color: #ffcc80; }
  .state-ROUNDABOUT { background: #4a148c; color: #ce93d8; }
  .state-BOOM_GATE_1, .state-BOOM_GATE_2 { background: #b71c1c; color: #ef9a9a; }
  .state-TUNNEL { background: #263238; color: #90a4ae; }
  .state-HILL { background: #33691e; color: #aed581; }
  .state-BUMPER { background: #795548; color: #d7ccc8; }
  .state-TRAFFIC_LIGHT { background: #f57f17; color: #fff9c4; }
  .state-PARALLEL_PARK, .state-PERPENDICULAR_PARK { background: #0d47a1; color: #90caf9; }
  .state-DRIVE_TO_PERP { background: #1565c0; color: #90caf9; }
  .state-FINISHED { background: #ffd600; color: #333; }

  .mode-badge {
    padding: 6px 16px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 1.1em;
  }
  .mode-AUTO { background: #4caf50; color: #fff; }
  .mode-MANUAL { background: #ff9800; color: #fff; }

  .lap-badge {
    padding: 6px 14px;
    border-radius: 6px;
    font-weight: 600;
    font-size: 1em;
    background: #0f3460;
    color: #e94560;
    margin-left: 10px;
  }

  /* Sensor indicators */
  .sensor-row {
    display: flex;
    align-items: center;
    padding: 6px 0;
    border-bottom: 1px solid #ffffff08;
  }
  .sensor-row:last-child { border: none; }
  .sensor-label { flex: 1; font-size: 0.9em; color: #aaa; }
  .sensor-value { font-weight: 600; font-size: 0.95em; }
  .dot {
    width: 12px; height: 12px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 6px;
  }
  .dot-green { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
  .dot-red { background: #f44336; box-shadow: 0 0 6px #f44336; }
  .dot-yellow { background: #ffeb3b; box-shadow: 0 0 6px #ffeb3b; }
  .dot-gray { background: #555; }

  /* Traffic light */
  .traffic-light {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
    background: #111;
    padding: 10px 14px;
    border-radius: 10px;
    width: fit-content;
    margin: 0 auto;
  }
  .tl-circle {
    width: 32px; height: 32px;
    border-radius: 50%;
    border: 2px solid #333;
    transition: all 0.3s;
  }
  .tl-red    { background: #331111; }
  .tl-yellow { background: #332b11; }
  .tl-green  { background: #113311; }
  .tl-red.active    { background: #f44336; box-shadow: 0 0 16px #f44336; border-color: #f44336; }
  .tl-yellow.active { background: #ffeb3b; box-shadow: 0 0 16px #ffeb3b; border-color: #ffeb3b; }
  .tl-green.active  { background: #4caf50; box-shadow: 0 0 16px #4caf50; border-color: #4caf50; }

  /* Controller */
  .controller {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 6px;
    text-align: center;
  }
  .ctrl-btn {
    padding: 8px;
    border-radius: 6px;
    background: #222;
    font-size: 0.8em;
    color: #666;
    transition: all 0.2s;
  }
  .ctrl-btn.active {
    background: #e94560;
    color: #fff;
    box-shadow: 0 0 8px #e94560;
  }

  /* Meters */
  .meter-bar {
    height: 8px;
    background: #222;
    border-radius: 4px;
    overflow: hidden;
    margin-top: 4px;
  }
  .meter-fill {
    height: 100%;
    border-radius: 4px;
    transition: width 0.3s;
  }
  .meter-fill.blue { background: linear-gradient(90deg, #1565c0, #42a5f5); }
  .meter-fill.orange { background: linear-gradient(90deg, #e65100, #ff9800); }

  /* Camera */
  .camera-feed {
    width: 100%;
    border-radius: 8px;
    background: #111;
    min-height: 180px;
    object-fit: contain;
  }

  .odom-val {
    font-size: 1.8em;
    font-weight: 700;
    color: #42a5f5;
  }
  .odom-unit {
    font-size: 0.7em;
    color: #888;
    margin-left: 4px;
  }

  @media (max-width: 768px) {
    .grid { grid-template-columns: 1fr; }
    .card.wide, .card.full { grid-column: span 1; }
  }
</style>
</head>
<body>
<div class="header">
  <h1>🤖 RISA-Bot Dashboard</h1>
  <div><span class="status-dot" id="connDot"></span><span id="connText" style="font-size:0.85em;color:#888;">Connecting...</span></div>
</div>

<div class="grid">
  <!-- State & Mode -->
  <div class="card wide">
    <h3>State Machine</h3>
    <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap;">
      <span class="state-badge" id="stateBadge">—</span>
      <span class="mode-badge" id="modeBadge">—</span>
      <span class="lap-badge" id="lapBadge">Lap —</span>
    </div>
    <div style="margin-top:10px;display:flex;gap:20px;font-size:0.85em;color:#888;">
      <span>⏱ <span id="stateTime">0</span>s in state</span>
      <span>📏 <span id="stateDist">0.00</span>m in state</span>
    </div>
  </div>

  <!-- Traffic Light -->
  <div class="card">
    <h3>Traffic Light</h3>
    <div class="traffic-light">
      <div class="tl-circle tl-red" id="tlRed"></div>
      <div class="tl-circle tl-yellow" id="tlYellow"></div>
      <div class="tl-circle tl-green" id="tlGreen"></div>
    </div>
    <div style="text-align:center;margin-top:8px;font-size:0.85em;color:#888;" id="tlText">unknown</div>
  </div>

  <!-- Sensors -->
  <div class="card">
    <h3>Sensors</h3>
    <div class="sensor-row">
      <span class="sensor-label">LiDAR Obstacle</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotLidar"></span><span id="valLidar">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Camera Obstacle</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotCam"></span><span id="valCam">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Fused Obstacle</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotFused"></span><span id="valFused">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Boom Gate</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotGate"></span><span id="valGate">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Tunnel</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotTunnel"></span><span id="valTunnel">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Obstruction</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotObst"></span><span id="valObst">—</span></span>
    </div>
    <div class="sensor-row">
      <span class="sensor-label">Parking Done</span>
      <span class="sensor-value"><span class="dot dot-gray" id="dotPark"></span><span id="valPark">—</span></span>
    </div>
  </div>

  <!-- Lane Error + Steering -->
  <div class="card">
    <h3>Lane Following</h3>
    <div class="sensor-row">
      <span class="sensor-label">Lane Error</span>
      <span class="sensor-value" id="laneErr">0.00</span>
    </div>
    <div class="meter-bar"><div class="meter-fill blue" id="laneBar" style="width:50%"></div></div>
    <div style="margin-top:10px;">
      <div class="sensor-row">
        <span class="sensor-label">Cmd Linear X</span>
        <span class="sensor-value" id="cmdLinX">0.00</span>
      </div>
      <div class="sensor-row">
        <span class="sensor-label">Cmd Angular Z</span>
        <span class="sensor-value" id="cmdAngZ">0.00</span>
      </div>
    </div>
  </div>

  <!-- Odometry -->
  <div class="card">
    <h3>Odometry</h3>
    <div class="odom-val"><span id="odomDist">0.00</span><span class="odom-unit">m</span></div>
    <div style="margin-top:6px;">
      <div class="sensor-row">
        <span class="sensor-label">Speed</span>
        <span class="sensor-value" id="odomSpeed">0.00 m/s</span>
      </div>
    </div>
  </div>

  <!-- Speed & State Cycle -->
  <div class="card">
    <h3>Manual Control</h3>
    <div class="sensor-row">
      <span class="sensor-label">Speed</span>
      <span class="sensor-value" style="color:#42a5f5;font-size:1.4em;font-weight:700;" id="speedPct">50%</span>
    </div>
    <div class="meter-bar"><div class="meter-fill orange" id="speedBar" style="width:50%"></div></div>
    <div style="margin-top:10px;font-size:0.8em;color:#666;">D-pad ▲/▼ to adjust ±10%</div>
    <div style="margin-top:12px;">
      <div class="sensor-row">
        <span class="sensor-label">Challenge Selector</span>
        <span class="sensor-value" id="ctrlState" style="color:#ce93d8;">LANE_FOLLOW</span>
      </div>
      <div style="font-size:0.8em;color:#666;">LB/RB to cycle states</div>
    </div>
  </div>

  <!-- Controller -->
  <div class="card">
    <h3>Controller</h3>
    <div class="controller">
      <div class="ctrl-btn" id="btnLB">LB</div>
      <div class="ctrl-btn" id="btnUp">▲</div>
      <div class="ctrl-btn" id="btnRB">RB</div>
      <div class="ctrl-btn" id="btnLeft">◀</div>
      <div class="ctrl-btn" id="btnStart">START</div>
      <div class="ctrl-btn" id="btnRight">▶</div>
      <div class="ctrl-btn" id="btnX">X</div>
      <div class="ctrl-btn" id="btnDown">▼</div>
      <div class="ctrl-btn" id="btnY">Y</div>
      <div class="ctrl-btn" id="btnA">A</div>
      <div class="ctrl-btn" id="btnLT">LT</div>
      <div class="ctrl-btn" id="btnB">B</div>
      <div class="ctrl-btn" id="btnRT" style="grid-column:3;">RT</div>
    </div>
    <div style="margin-top:8px;font-size:0.8em;color:#666;display:flex;gap:10px;">
      <span>L: <span id="joyL">0.0, 0.0</span></span>
      <span>R: <span id="joyR">0.0, 0.0</span></span>
    </div>
  </div>
</div>

<script>
function update() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      document.getElementById('connDot').style.background = '#4caf50';
      document.getElementById('connText').textContent = 'Connected';

      // State
      const sb = document.getElementById('stateBadge');
      sb.textContent = d.state;
      sb.className = 'state-badge state-' + d.state;

      const mb = document.getElementById('modeBadge');
      mb.textContent = d.auto_mode ? 'AUTO' : 'MANUAL';
      mb.className = 'mode-badge mode-' + (d.auto_mode ? 'AUTO' : 'MANUAL');

      document.getElementById('lapBadge').textContent = 'Lap ' + d.lap;
      document.getElementById('stateTime').textContent = d.state_time;
      document.getElementById('stateDist').textContent = d.state_dist;

      // Traffic light
      ['Red','Yellow','Green'].forEach(c => {
        const el = document.getElementById('tl' + c);
        el.classList.toggle('active', d.traffic_light === c.toLowerCase());
      });
      document.getElementById('tlText').textContent = d.traffic_light;

      // Sensors
      function setSensor(dotId, valId, val, trueLabel, falseLabel) {
        const dot = document.getElementById(dotId);
        const v = document.getElementById(valId);
        if (val === null || val === undefined) { dot.className = 'dot dot-gray'; v.textContent = '—'; return; }
        dot.className = val ? 'dot dot-red' : 'dot dot-green';
        v.textContent = val ? (trueLabel||'YES') : (falseLabel||'NO');
      }
      setSensor('dotLidar','valLidar', d.lidar_obstacle, 'BLOCKED','CLEAR');
      setSensor('dotCam','valCam', d.camera_obstacle, 'BLOCKED','CLEAR');
      setSensor('dotFused','valFused', d.fused_obstacle, 'BLOCKED','CLEAR');
      setSensor('dotTunnel','valTunnel', d.tunnel_detected, 'IN TUNNEL','NO');
      setSensor('dotObst','valObst', d.obstruction_active, 'DODGING','NO');
      setSensor('dotPark','valPark', d.parking_complete, 'DONE','NO');

      const gDot = document.getElementById('dotGate');
      const gVal = document.getElementById('valGate');
      if (d.boom_gate === null) { gDot.className='dot dot-gray'; gVal.textContent='—'; }
      else { gDot.className = d.boom_gate ? 'dot dot-green' : 'dot dot-red'; gVal.textContent = d.boom_gate ? 'OPEN' : 'CLOSED'; }

      // Lane
      document.getElementById('laneErr').textContent = d.lane_error.toFixed(3);
      const pct = Math.min(Math.max((d.lane_error + 0.5) / 1.0 * 100, 0), 100);
      document.getElementById('laneBar').style.width = pct + '%';

      // Cmd vel
      document.getElementById('cmdLinX').textContent = d.cmd_lin_x.toFixed(3);
      document.getElementById('cmdAngZ').textContent = d.cmd_ang_z.toFixed(3);

      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3) + ' m/s';

      // Speed & State Cycle
      document.getElementById('speedPct').textContent = d.speed_pct + '%';
      document.getElementById('speedBar').style.width = d.speed_pct + '%';
      document.getElementById('ctrlState').textContent = d.ctrl_state_name;

      // Controller buttons (correct mapping: A=0,B=1,X=3,Y=4,LB=6,RB=7,LT=8,RT=9,Start=11)
      const btnMap = {0:'btnA', 1:'btnB', 3:'btnX', 4:'btnY', 6:'btnLB', 7:'btnRB', 8:'btnLT', 9:'btnRT', 11:'btnStart'};
      Object.values(btnMap).forEach(id => document.getElementById(id).classList.remove('active'));
      if (d.buttons) d.buttons.forEach((v,i) => { if (v && btnMap[i]) document.getElementById(btnMap[i]).classList.add('active'); });

      // D-pad from axes
      if (d.axes && d.axes.length > 7) {
        document.getElementById('btnLeft').classList.toggle('active', d.axes[6] > 0.5);
        document.getElementById('btnRight').classList.toggle('active', d.axes[6] < -0.5);
        document.getElementById('btnUp').classList.toggle('active', d.axes[7] > 0.5);
        document.getElementById('btnDown').classList.toggle('active', d.axes[7] < -0.5);
      }

      if (d.axes && d.axes.length > 3) {
        document.getElementById('joyL').textContent = d.axes[0].toFixed(1)+', '+d.axes[1].toFixed(1);
        document.getElementById('joyR').textContent = d.axes[2].toFixed(1)+', '+d.axes[3].toFixed(1);
      }
    })
    .catch(() => {
      document.getElementById('connDot').style.background = '#f44336';
      document.getElementById('connText').textContent = 'Disconnected';
    });
}
setInterval(update, 200);
update();
</script>
</body>
</html>"""


# ======================== ROS2 Dashboard Node ========================

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.get_logger().info('Dashboard node starting on http://0.0.0.0:8080')

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
            'lane_error': 0.0,
            'cmd_lin_x': 0.0,
            'cmd_ang_z': 0.0,
            'distance': 0.0,
            'speed': 0.0,
            'buttons': [],
            'axes': [],
            'speed_pct': 50,
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

    def _set(self, key, value):
        with self.data_lock:
            self.data[key] = value

    def _auto_mode_cb(self, msg): self._set('auto_mode', msg.data)
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

    def _odom_cb(self, msg):
        with self.data_lock:
            self.data['speed'] = msg.twist.twist.linear.x

    def _joy_cb(self, msg):
        with self.data_lock:
            self.data['buttons'] = list(msg.buttons)
            self.data['axes'] = list(msg.axes)

    def _dash_state_cb(self, msg):
        """Parse STATE|LAP|DIST format from auto_driver."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                self.data['state'] = parts[0]
                if len(parts) > 1:
                    self.data['lap'] = int(parts[1])
                if len(parts) > 2:
                    self.data['state_dist'] = parts[2]
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

    def get_json(self):
        with self.data_lock:
            d = dict(self.data)
        d['state_time'] = int(time.time() - self._state_entry_time)
        return json.dumps(d)


# ======================== HTTP Server ========================

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
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


def main(args=None):
    global _node_ref
    rclpy.init(args=args)
    node = DashboardNode()
    _node_ref = node

    # Start HTTP server in background thread
    server = http.server.HTTPServer(('0.0.0.0', 8080), DashboardHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    node.get_logger().info('Dashboard live at http://0.0.0.0:8080')

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
