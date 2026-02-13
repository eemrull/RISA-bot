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
import subprocess
import numpy as np
import cv2

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

# ======================== HTML Dashboard ========================

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RISA-Bot Dashboard</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&display=swap" rel="stylesheet">
<style>
  :root {
    --bg: #0a0a14;
    --card: rgba(18,18,35,0.85);
    --card-border: rgba(255,255,255,0.06);
    --accent: #e94560;
    --accent2: #0f3460;
    --text: #e0e0e0;
    --muted: #555;
    --radius: 16px;
    --glow: rgba(233,69,96,0.15);
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: var(--bg);
    background-image:
      radial-gradient(ellipse at 20% 50%, rgba(15,52,96,0.2) 0%, transparent 50%),
      radial-gradient(ellipse at 80% 20%, rgba(233,69,96,0.12) 0%, transparent 50%),
      radial-gradient(ellipse at 50% 100%, rgba(66,165,245,0.06) 0%, transparent 40%);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* ===== HEADER ===== */
  .header {
    background: rgba(10,10,25,0.95);
    backdrop-filter: blur(24px);
    padding: 14px 28px;
    border-bottom: 1px solid rgba(233,69,96,0.15);
    display: flex;
    align-items: center;
    justify-content: space-between;
    position: sticky;
    top: 0;
    z-index: 100;
    box-shadow: 0 4px 30px rgba(0,0,0,0.4);
  }
  .header h1 {
    font-size: 1.4em;
    font-weight: 800;
    background: linear-gradient(135deg, var(--accent), #ff6b81, #42a5f5);
    background-size: 200% 200%;
    animation: gradShift 4s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    letter-spacing: -0.5px;
  }
  @keyframes gradShift { 0%,100%{background-position:0% 50%} 50%{background-position:100% 50%} }
  .conn-badge {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 0.8em;
    color: var(--muted);
  }
  .conn-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: #4caf50;
    animation: pulse 2s infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.3} }

  /* ===== LAYOUT ===== */
  .layout {
    display: grid;
    grid-template-columns: 260px 1fr 260px;
    gap: 14px;
    padding: 14px;
    max-width: 1400px;
    margin: 0 auto;
    min-height: calc(100vh - 50px);
  }
  .col { display: flex; flex-direction: column; gap: 12px; }

  /* ===== CARDS ===== */
  .card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 18px;
    transition: all 0.35s cubic-bezier(0.4,0,0.2,1);
    position: relative;
    overflow: hidden;
  }
  .card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0;
    transition: opacity 0.3s;
  }
  .card:hover {
    border-color: rgba(233,69,96,0.2);
    box-shadow: 0 4px 20px rgba(233,69,96,0.08);
    transform: translateY(-1px);
  }
  .card:hover::before { opacity: 1; }
  .card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 12px;
    font-weight: 700;
  }

  /* ===== STATE BADGE ===== */
  .state-badge {
    display: inline-block;
    padding: 6px 14px;
    border-radius: 8px;
    font-size: 1em;
    font-weight: 700;
    letter-spacing: 0.5px;
    transition: all 0.3s;
  }
  .state-LANE_FOLLOW { background: #1b5e20; color: #a5d6a7; }
  .state-OBSTRUCTION { background: #e65100; color: #ffcc80; }
  .state-ROUNDABOUT { background: #4a148c; color: #ce93d8; }
  .state-BOOM_GATE_1,.state-BOOM_GATE_2 { background: #b71c1c; color: #ef9a9a; }
  .state-TUNNEL { background: #263238; color: #90a4ae; }
  .state-HILL { background: #33691e; color: #aed581; }
  .state-BUMPER { background: #795548; color: #d7ccc8; }
  .state-TRAFFIC_LIGHT { background: #f57f17; color: #fff9c4; }
  .state-PARALLEL_PARK,.state-PERPENDICULAR_PARK { background: #0d47a1; color: #90caf9; }
  .state-DRIVE_TO_PERP { background: #1565c0; color: #90caf9; }
  .state-FINISHED { background: #ffd600; color: #333; }

  .mode-badge {
    padding: 4px 12px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 0.85em;
    transition: all 0.3s;
  }
  .mode-AUTO { background: #4caf50; color: #fff; box-shadow: 0 0 12px rgba(76,175,80,0.4); }
  .mode-MANUAL { background: #ff9800; color: #fff; box-shadow: 0 0 12px rgba(255,152,0,0.4); }

  .lap-badge {
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 600;
    font-size: 0.8em;
    background: var(--accent2);
    color: var(--accent);
  }
  .info-row {
    display: flex;
    gap: 16px;
    margin-top: 8px;
    font-size: 0.75em;
    color: var(--muted);
  }

  /* ===== TRAFFIC LIGHT ===== */
  .traffic-light {
    display: flex;
    gap: 8px;
    align-items: center;
    justify-content: center;
    background: #0a0a0a;
    padding: 10px 20px;
    border-radius: 25px;
    width: fit-content;
    margin: 0 auto;
  }
  .tl-circle {
    width: 28px; height: 28px;
    border-radius: 50%;
    border: 2px solid #222;
    transition: all 0.3s;
  }
  .tl-red    { background: #221111; }
  .tl-yellow { background: #222011; }
  .tl-green  { background: #112211; }
  .tl-red.active    { background: #f44336; box-shadow: 0 0 20px #f44336; border-color: #f44336; }
  .tl-yellow.active { background: #ffeb3b; box-shadow: 0 0 20px #ffeb3b; border-color: #ffeb3b; }
  .tl-green.active  { background: #4caf50; box-shadow: 0 0 20px #4caf50; border-color: #4caf50; }
  .tl-label { text-align: center; margin-top: 6px; font-size: 0.75em; color: var(--muted); }

  /* ===== SENSOR ROWS ===== */
  .s-row {
    display: flex;
    align-items: center;
    padding: 5px 0;
    border-bottom: 1px solid rgba(255,255,255,0.03);
  }
  .s-row:last-child { border: none; }
  .s-label { flex: 1; font-size: 0.78em; color: #999; }
  .s-val { font-weight: 600; font-size: 0.82em; }
  .dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 5px;
    transition: all 0.3s;
  }
  .dot-green { background: #4caf50; box-shadow: 0 0 6px #4caf50; }
  .dot-red { background: #f44336; box-shadow: 0 0 6px #f44336; }
  .dot-gray { background: #333; }

  /* ===== METER BARS ===== */
  .meter { height: 8px; background: rgba(26,26,46,0.8); border-radius: 4px; overflow: hidden; margin-top: 6px; box-shadow: inset 0 1px 3px rgba(0,0,0,0.3); }
  .meter-fill { height: 100%; border-radius: 4px; transition: width 0.4s cubic-bezier(0.4,0,0.2,1), background 0.4s; position: relative; }
  .meter-fill::after { content: ''; position: absolute; top: 0; left: 0; right: 0; height: 50%; background: linear-gradient(180deg, rgba(255,255,255,0.15), transparent); border-radius: 4px 4px 0 0; }
  .meter-blue { background: linear-gradient(90deg, #1565c0, #42a5f5); }
  .meter-orange { background: linear-gradient(90deg, #e65100, #ff9800, #ffb74d); }

  /* ===== SPEED GAUGE ===== */
  .speed-display {
    text-align: center;
    padding: 12px 0;
  }
  .speed-display .num {
    font-size: 2.8em;
    font-weight: 800;
    background: linear-gradient(135deg, #42a5f5, #1e88e5, #e94560);
    background-size: 200% 200%;
    animation: gradShift 3s ease infinite;
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    line-height: 1;
    transition: all 0.3s;
  }
  .speed-display .unit {
    font-size: 0.6em;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 3px;
    margin-top: 4px;
  }
  .gear-dots {
    display: flex; justify-content: center; gap: 6px; margin-top: 8px;
  }
  .gear-dot {
    width: 10px; height: 10px; border-radius: 50%; background: rgba(255,255,255,0.08);
    border: 1px solid rgba(255,255,255,0.1);
    transition: all 0.3s;
  }
  .gear-dot.active {
    background: var(--accent); border-color: var(--accent);
    box-shadow: 0 0 8px rgba(233,69,96,0.5);
  }

  /* ===== CHALLENGE SELECTOR ===== */
  .ctrl-state-badge {
    display: inline-block;
    padding: 4px 10px;
    border-radius: 6px;
    font-weight: 600;
    font-size: 0.78em;
    background: rgba(74,20,140,0.4);
    color: #ce93d8;
    border: 1px solid rgba(206,147,216,0.2);
  }

  /* ===== CAMERA ===== */
  .cam-card {
    display: flex;
    flex-direction: column;
  }
  .cam-toggle {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 8px 16px;
    border-radius: 8px;
    border: 1px solid rgba(255,255,255,0.1);
    background: rgba(255,255,255,0.05);
    color: #42a5f5;
    cursor: pointer;
    font-size: 0.8em;
    font-weight: 600;
    transition: all 0.2s;
    margin-bottom: 10px;
    font-family: inherit;
  }
  .cam-toggle:hover { background: rgba(66,165,245,0.15); border-color: #42a5f5; }
  .cam-toggle.on { background: rgba(76,175,80,0.2); color: #4caf50; border-color: #4caf50; }
  .cam-container {
    width: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #050508;
    border-radius: 12px;
    overflow: hidden;
    position: relative;
  }
  .cam-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    border-radius: 8px;
  }
  .cam-off {
    color: #333;
    font-size: 0.85em;
    text-align: center;
  }
  .cam-off .icon { font-size: 3em; margin-bottom: 8px; opacity: 0.3; }

  /* ===== CONTROLLER ===== */
  .ctrl-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 4px;
    text-align: center;
  }
  .ctrl-btn {
    padding: 6px 2px;
    border-radius: 6px;
    background: rgba(255,255,255,0.04);
    font-size: 0.7em;
    color: #444;
    transition: all 0.15s;
    font-weight: 500;
  }
  .ctrl-btn.active {
    background: var(--accent);
    color: #fff;
    box-shadow: 0 0 10px rgba(233,69,96,0.4);
  }
  .joy-info {
    margin-top: 6px;
    font-size: 0.7em;
    color: #444;
    display: flex;
    gap: 8px;
  }
  .btn-debug {
    margin-top: 6px;
    padding: 4px 6px;
    background: rgba(0,0,0,0.3);
    border-radius: 4px;
    font-size: 0.65em;
    font-family: 'Courier New', monospace;
    color: #ff9800;
    min-height: 18px;
  }

  /* ===== ODOM ===== */
  .odom-big {
    font-size: 1.6em;
    font-weight: 700;
    color: #42a5f5;
  }
  .odom-unit { font-size: 0.5em; color: var(--muted); margin-left: 3px; }

  /* ===== RESPONSIVE ===== */
  @media (max-width: 960px) {
    .layout { grid-template-columns: 1fr; }
    .cam-container { min-height: 200px; }
    .flow-bar { overflow-x: auto; }
  }

  /* ===== COMPETITION FLOW ===== */
  .flow-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .flow-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 18px;
    position: relative;
    overflow: hidden;
  }
  .flow-card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, #42a5f5, transparent);
    opacity: 0.5;
  }
  .flow-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .flow-bar {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 6px;
    padding: 8px 0;
  }
  .flow-node {
    padding: 6px 10px;
    border-radius: 8px;
    font-size: 0.68em;
    font-weight: 600;
    letter-spacing: 0.2px;
    background: rgba(255,255,255,0.04);
    border: 1px solid rgba(255,255,255,0.06);
    color: #555;
    transition: all 0.4s cubic-bezier(0.4,0,0.2,1);
    cursor: default;
    white-space: nowrap;
    position: relative;
  }
  .flow-node.done {
    background: rgba(76,175,80,0.12);
    border-color: rgba(76,175,80,0.3);
    color: #81c784;
  }
  .flow-node.done::after {
    content: '✓';
    position: absolute;
    top: -4px; right: -4px;
    width: 14px; height: 14px;
    background: #4caf50;
    border-radius: 50%;
    font-size: 9px;
    display: flex; align-items: center; justify-content: center;
    color: #fff;
  }
  .flow-node.active {
    background: linear-gradient(135deg, rgba(233,69,96,0.2), rgba(66,165,245,0.2));
    border-color: var(--accent);
    color: #fff;
    box-shadow: 0 0 20px rgba(233,69,96,0.25), 0 0 40px rgba(233,69,96,0.1);
    transform: scale(1.08);
    font-weight: 700;
  }
  @keyframes activeGlow {
    0%,100% { box-shadow: 0 0 15px rgba(233,69,96,0.2); }
    50% { box-shadow: 0 0 25px rgba(233,69,96,0.4), 0 0 50px rgba(233,69,96,0.15); }
  }
  .flow-node.active { animation: activeGlow 2s ease infinite; }
  .flow-arrow {
    color: #333;
    font-size: 0.6em;
    padding: 0 1px;
    transition: color 0.3s;
  }
  .flow-arrow.passed { color: #4caf50; }
  .flow-lap-label {
    flex-shrink: 0;
    padding: 4px 10px;
    border-radius: 12px;
    font-size: 0.55em;
    font-weight: 700;
    letter-spacing: 1px;
    text-transform: uppercase;
    margin: 0 6px;
  }
  .lap1-label { background: rgba(233,69,96,0.15); color: var(--accent); border: 1px solid rgba(233,69,96,0.2); }
  .lap2-label { background: rgba(66,165,245,0.15); color: #42a5f5; border: 1px solid rgba(66,165,245,0.2); }

  /* ===== CONTROLLER POPOUT ===== */
  .ctrl-popout-tab {
    position: fixed;
    right: 0;
    top: 50%;
    transform: translateY(-50%);
    background: rgba(18,18,35,0.95);
    backdrop-filter: blur(16px);
    padding: 10px 8px;
    border-radius: 10px 0 0 10px;
    border: 1px solid var(--card-border);
    border-right: none;
    cursor: pointer;
    z-index: 200;
    writing-mode: vertical-rl;
    text-orientation: mixed;
    font-size: 0.7em;
    font-weight: 600;
    color: var(--accent);
    letter-spacing: 1px;
    transition: all 0.3s;
  }
  .ctrl-popout-tab:hover {
    background: rgba(233,69,96,0.15);
    padding-right: 12px;
  }
  .ctrl-drawer {
    position: fixed;
    right: -320px;
    top: 60px;
    bottom: 0;
    width: 300px;
    background: rgba(10,10,25,0.97);
    backdrop-filter: blur(20px);
    border-left: 1px solid rgba(233,69,96,0.15);
    z-index: 199;
    transition: right 0.4s cubic-bezier(0.4,0,0.2,1);
    overflow-y: auto;
    padding: 20px;
    box-shadow: -4px 0 30px rgba(0,0,0,0.5);
  }
  .ctrl-drawer.open { right: 0; }
  .ctrl-drawer h3 {
    font-size: 0.7em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: var(--accent);
    margin-bottom: 14px;
    font-weight: 700;
  }
  .ctrl-map-row {
    display: flex;
    align-items: center;
    padding: 8px 0;
    border-bottom: 1px solid rgba(255,255,255,0.04);
  }
  .ctrl-map-row:last-child { border: none; }
  .ctrl-key {
    flex-shrink: 0;
    width: 60px;
    padding: 4px 8px;
    font-size: 0.75em;
    font-weight: 700;
    text-align: center;
    border-radius: 6px;
    background: rgba(255,255,255,0.06);
    border: 1px solid rgba(255,255,255,0.1);
    color: #aaa;
    margin-right: 12px;
  }
  .ctrl-desc {
    font-size: 0.78em;
    color: #888;
  }
  .ctrl-section-label {
    font-size: 0.6em;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: #444;
    margin: 16px 0 8px;
    padding-bottom: 4px;
    border-bottom: 1px solid rgba(255,255,255,0.05);
  }

  /* ===== EVENT LOG ===== */
  .log-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .log-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 14px 18px;
  }
  .log-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 10px;
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .log-scroll {
    max-height: 100px;
    overflow-y: auto;
    scrollbar-width: thin;
    scrollbar-color: #222 transparent;
  }
  .log-entry {
    font-size: 0.72em;
    padding: 3px 0;
    color: #555;
    font-family: 'Courier New', monospace;
    border-bottom: 1px solid rgba(255,255,255,0.02);
    animation: logFade 0.5s ease;
  }
  .log-entry:first-child { color: #aaa; }
  @keyframes logFade { from { opacity: 0; transform: translateX(-10px); } to { opacity: 1; transform: translateX(0); } }
  .log-time { color: #42a5f5; margin-right: 8px; }
  .log-event { color: #aaa; }
  .log-val { color: var(--accent); font-weight: 600; }

  /* ===== PARAMETER PANEL ===== */
  .param-section {
    max-width: 1400px;
    margin: 0 auto;
    padding: 0 14px 14px;
  }
  .param-card {
    background: var(--card);
    backdrop-filter: blur(16px);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 18px;
  }
  .param-card h3 {
    font-size: 0.65em;
    text-transform: uppercase;
    letter-spacing: 2.5px;
    color: var(--muted);
    margin-bottom: 12px;
    font-weight: 700;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .param-toolbar {
    display: flex; gap: 8px; align-items: center;
    margin-bottom: 12px;
  }
  .param-refresh-btn, .param-load-btn {
    padding: 5px 14px;
    border-radius: 6px;
    border: 1px solid rgba(255,255,255,0.1);
    background: rgba(255,255,255,0.05);
    color: #42a5f5;
    cursor: pointer;
    font-size: 0.75em;
    font-weight: 600;
    transition: all 0.2s;
    font-family: inherit;
  }
  .param-refresh-btn:hover, .param-load-btn:hover {
    background: rgba(66,165,245,0.15); border-color: #42a5f5;
  }
  .param-node-block {
    margin-bottom: 12px;
    border: 1px solid rgba(255,255,255,0.04);
    border-radius: 10px;
    overflow: hidden;
  }
  .param-node-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 14px;
    background: rgba(255,255,255,0.03);
    cursor: pointer;
    transition: background 0.2s;
  }
  .param-node-header:hover { background: rgba(255,255,255,0.06); }
  .param-node-name {
    font-size: 0.8em;
    font-weight: 700;
    color: #42a5f5;
  }
  .param-node-count {
    font-size: 0.65em;
    color: #444;
  }
  .param-node-body {
    max-height: 0;
    overflow: hidden;
    transition: max-height 0.35s ease;
  }
  .param-node-body.open {
    max-height: 2000px;
  }
  .param-row {
    display: flex;
    align-items: center;
    padding: 6px 14px;
    border-top: 1px solid rgba(255,255,255,0.03);
    gap: 8px;
  }
  .param-name {
    flex: 1;
    font-size: 0.72em;
    color: #888;
    font-family: 'Courier New', monospace;
    word-break: break-all;
  }
  .param-val {
    width: 120px;
    padding: 4px 8px;
    border-radius: 4px;
    border: 1px solid rgba(255,255,255,0.1);
    background: rgba(0,0,0,0.3);
    color: #e0e0e0;
    font-size: 0.72em;
    font-family: 'Courier New', monospace;
    outline: none;
    transition: border-color 0.2s;
  }
  .param-val:focus { border-color: var(--accent); }
  .param-set-btn {
    padding: 4px 10px;
    border-radius: 4px;
    border: 1px solid rgba(76,175,80,0.3);
    background: rgba(76,175,80,0.1);
    color: #81c784;
    cursor: pointer;
    font-size: 0.65em;
    font-weight: 600;
    transition: all 0.2s;
    font-family: inherit;
  }
  .param-set-btn:hover { background: rgba(76,175,80,0.25); border-color: #4caf50; }
  .param-status {
    font-size: 0.65em;
    padding: 2px 6px;
    border-radius: 3px;
    animation: logFade 0.3s ease;
  }
  .param-status.ok { color: #4caf50; }
  .param-status.err { color: #f44336; }
  .param-empty {
    text-align: center;
    padding: 20px;
    color: #333;
    font-size: 0.8em;
  }
</style>
</head>
<body>

<!-- HEADER -->
<div class="header">
  <h1>🤖 RISA-Bot <span style="font-size:0.6em;color:rgba(255,255,255,0.4);vertical-align:middle;">v1.0</span></h1>
  <div class="conn-badge">
    <span class="conn-dot" id="connDot"></span>
    <span id="connText">Connecting...</span>
  </div>
</div>

<!-- MAIN LAYOUT -->
<div class="layout">

  <!-- ===== LEFT COLUMN ===== -->
  <div class="col">

    <!-- State Machine -->
    <div class="card">
      <h3>State Machine</h3>
      <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap;">
        <span class="state-badge" id="stateBadge">—</span>
        <span class="mode-badge" id="modeBadge">MANUAL</span>
      </div>
      <div style="margin-top:8px;">
        <span class="lap-badge" id="lapBadge">Lap —</span>
      </div>
      <div class="info-row">
        <span>⏱ <span id="stateTime">0</span>s</span>
        <span>📏 <span id="stateDist">0.00</span>m</span>
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
      <div class="tl-label" id="tlText">unknown</div>
    </div>

    <!-- Manual Control -->
    <div class="card">
      <h3>Manual Control</h3>
      <div class="speed-display">
        <div class="num" id="speedPct">40%</div>
        <div class="unit">Drive Speed</div>
        <div class="gear-dots">
          <div class="gear-dot" id="gear0"></div>
          <div class="gear-dot active" id="gear1"></div>
          <div class="gear-dot" id="gear2"></div>
          <div class="gear-dot" id="gear3"></div>
        </div>
      </div>
      <div class="meter"><div class="meter-fill meter-orange" id="speedBar" style="width:40%"></div></div>
      <div style="margin-top:6px;font-size:0.65em;color:#444;text-align:center;">D-pad ▲/▼ to shift</div>
      <div style="margin-top:12px;">
        <div class="s-row">
          <span class="s-label">Selector</span>
          <span class="ctrl-state-badge" id="ctrlState">LANE_FOLLOW</span>
        </div>
        <div style="font-size:0.65em;color:#444;">LB / RB to cycle</div>
      </div>
    </div>

    <!-- Lane Following -->
    <div class="card">
      <h3>Lane Following</h3>
      <div class="s-row">
        <span class="s-label">Error</span>
        <span class="s-val" id="laneErr">0.000</span>
      </div>
      <div class="meter"><div class="meter-fill meter-blue" id="laneBar" style="width:50%"></div></div>
      <div class="s-row" style="margin-top:6px;">
        <span class="s-label">Linear X</span>
        <span class="s-val" id="cmdLinX">0.000</span>
      </div>
      <div class="s-row">
        <span class="s-label">Angular Z</span>
        <span class="s-val" id="cmdAngZ">0.000</span>
      </div>
    </div>

  </div>

  <!-- ===== CENTER COLUMN — CAMERA ===== -->
  <div class="col">
    <div class="card cam-card">
      <h3>Camera Feed</h3>
      <button class="cam-toggle" id="camBtn" onclick="toggleCam()">📷 Enable Camera</button>
      <div class="cam-container" id="camContainer">
        <div class="cam-off" id="camOff">
          <div class="icon">📷</div>
          Click above to enable
        </div>
        <img id="camImg" src="" alt="Camera" style="display:none;">
      </div>
    </div>
  </div>

  <!-- ===== RIGHT COLUMN ===== -->
  <div class="col">

    <!-- Sensors -->
    <div class="card">
      <h3>Sensors</h3>
      <div class="s-row"><span class="s-label">LiDAR</span><span class="s-val"><span class="dot dot-gray" id="dotLidar"></span><span id="valLidar">—</span></span></div>
      <div class="s-row"><span class="s-label">Camera</span><span class="s-val"><span class="dot dot-gray" id="dotCam"></span><span id="valCam">—</span></span></div>
      <div class="s-row"><span class="s-label">Fused</span><span class="s-val"><span class="dot dot-gray" id="dotFused"></span><span id="valFused">—</span></span></div>
      <div class="s-row"><span class="s-label">Boom Gate</span><span class="s-val"><span class="dot dot-gray" id="dotGate"></span><span id="valGate">—</span></span></div>
      <div class="s-row"><span class="s-label">Tunnel</span><span class="s-val"><span class="dot dot-gray" id="dotTunnel"></span><span id="valTunnel">—</span></span></div>
      <div class="s-row"><span class="s-label">Obstruction</span><span class="s-val"><span class="dot dot-gray" id="dotObst"></span><span id="valObst">—</span></span></div>
      <div class="s-row"><span class="s-label">Parking</span><span class="s-val"><span class="dot dot-gray" id="dotPark"></span><span id="valPark">—</span></span></div>
    </div>

    <!-- Odometry -->
    <div class="card">
      <h3>Odometry</h3>
      <div class="odom-big"><span id="odomDist">0.00</span><span class="odom-unit">m</span></div>
      <div class="s-row" style="margin-top:6px;">
        <span class="s-label">Speed</span>
        <span class="s-val" id="odomSpeed">0.000 m/s</span>
      </div>
    </div>

    <!-- Controller -->
    <div class="card">
      <h3>Controller</h3>
      <div class="ctrl-grid">
        <div class="ctrl-btn" id="btnLB">LB</div>
        <div class="ctrl-btn" id="btnUp">▲</div>
        <div class="ctrl-btn" id="btnRB">RB</div>
        <div class="ctrl-btn" id="btnLeft">◀</div>
        <div class="ctrl-btn" id="btnStart">STA</div>
        <div class="ctrl-btn" id="btnRight">▶</div>
        <div class="ctrl-btn" id="btnX">X</div>
        <div class="ctrl-btn" id="btnDown">▼</div>
        <div class="ctrl-btn" id="btnY">Y</div>
        <div class="ctrl-btn" id="btnA">A</div>
        <div class="ctrl-btn" id="btnLT">LT</div>
        <div class="ctrl-btn" id="btnB">B</div>
        <div class="ctrl-btn" id="btnRT" style="grid-column:3;">RT</div>
      </div>
      <div class="joy-info">
        <span>L: <span id="joyL">0.0, 0.0</span></span>
        <span>R: <span id="joyR">0.0, 0.0</span></span>
      </div>
      <div class="btn-debug" id="btnDebug">Press a button to see index…</div>
    </div>

  </div>
</div>

<!-- ===== COMPETITION FLOW TIMELINE ===== -->
<div class="flow-section">
  <div class="flow-card">
    <h3>🏁 Competition Flow <span style="font-size:0.85em;color:#444;font-weight:400;text-transform:none;letter-spacing:0;"> — Lane following runs throughout all stages</span></h3>
    <div class="flow-bar" id="flowBar">
      <span class="flow-lap-label lap1-label">LAP 1</span>
      <div class="flow-node active" id="flow_OBSTRUCTION">Obstruction</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_ROUNDABOUT">Roundabout</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_BOOM_GATE_1">Gate 1</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_TUNNEL">Tunnel</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_BOOM_GATE_2">Gate 2</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_HILL">Hill</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_BUMPER">Bumper</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_TRAFFIC_LIGHT">Traffic Light</div>
      <span class="flow-arrow">▶</span>
      <span class="flow-lap-label lap2-label">LAP 2</span>
      <div class="flow-node" id="flow_PARALLEL_PARK">∥ Park</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_DRIVE_TO_PERP">Drive</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_PERPENDICULAR_PARK">⊥ Park</div>
      <span class="flow-arrow">▶</span>
      <div class="flow-node" id="flow_FINISHED">🏆 Done</div>
    </div>
  </div>
</div>

<!-- ===== EVENT LOG ===== -->
<div class="log-section">
  <div class="log-card">
    <h3><span style="opacity:0.6">📝</span> Event Log</h3>
    <div class="log-scroll" id="logScroll">
      <div class="log-entry"><span class="log-time">--:--:--</span><span class="log-event">Dashboard started</span></div>
    </div>
  </div>
</div>

<!-- ===== PARAMETER TUNING ===== -->
<div class="param-section">
  <div class="param-card">
    <h3><span>⚙️ Live Parameters</span></h3>
    <div class="param-toolbar">
      <button class="param-load-btn" onclick="loadAllParams()">Load Nodes</button>
      <button class="param-refresh-btn" onclick="refreshAllParams()">Refresh Values</button>
      <span style="font-size:0.65em;color:#333;margin-left:auto;" id="paramPoll"></span>
    </div>
    <div id="paramContainer">
      <div class="param-empty">Click "Load Nodes" to discover ROS2 parameters</div>
    </div>
  </div>
</div>

<!-- ===== CONTROLLER POPOUT ===== -->
<div class="ctrl-popout-tab" onclick="toggleCtrlDrawer()">🎮 Controls</div>
<div class="ctrl-drawer" id="ctrlDrawer">
  <h3>🎮 Control Mapping</h3>
  <div class="ctrl-section-label">Driving</div>
  <div class="ctrl-map-row"><span class="ctrl-key">L Stick Y</span><span class="ctrl-desc">Throttle (forward/reverse)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick X</span><span class="ctrl-desc">Steering (left/right)</span></div>
  <div class="ctrl-section-label">Speed</div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▲</span><span class="ctrl-desc">Speed up (shift gear)</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">D-Pad ▼</span><span class="ctrl-desc">Speed down (shift gear)</span></div>
  <div class="ctrl-section-label">Mode</div>
  <div class="ctrl-map-row"><span class="ctrl-key">Y</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">Start</span><span class="ctrl-desc">Toggle Auto/Manual mode</span></div>
  <div class="ctrl-section-label">Challenge</div>
  <div class="ctrl-map-row"><span class="ctrl-key">RB</span><span class="ctrl-desc">Next challenge state</span></div>
  <div class="ctrl-map-row"><span class="ctrl-key">LB</span><span class="ctrl-desc">Previous challenge state</span></div>
  <div class="ctrl-section-label">Camera</div>
  <div class="ctrl-map-row"><span class="ctrl-key">R Stick Y</span><span class="ctrl-desc">Camera tilt (if enabled)</span></div>
</div>

<script>
let camOn = false, camTimer = null;
let eventLog = [];
const FLOW_ORDER = ['OBSTRUCTION','ROUNDABOUT','BOOM_GATE_1','TUNNEL','BOOM_GATE_2','HILL','BUMPER','TRAFFIC_LIGHT','PARALLEL_PARK','DRIVE_TO_PERP','PERPENDICULAR_PARK','FINISHED'];
let lastState = '';

function toggleCtrlDrawer() {
  document.getElementById('ctrlDrawer').classList.toggle('open');
}

function addLogEntry(text) {
  const now = new Date();
  const ts = now.toLocaleTimeString('en-GB');
  eventLog.unshift({time: ts, text: text});
  if (eventLog.length > 30) eventLog.pop();
  const container = document.getElementById('logScroll');
  container.innerHTML = eventLog.map(e =>
    `<div class="log-entry"><span class="log-time">${e.time}</span><span class="log-event">${e.text}</span></div>`
  ).join('');
}

function updateFlow(currentState) {
  const activeIdx = FLOW_ORDER.indexOf(currentState);
  const arrows = document.querySelectorAll('.flow-arrow');
  FLOW_ORDER.forEach((s, i) => {
    const el = document.getElementById('flow_' + s);
    if (!el) return;
    el.classList.remove('active', 'done');
    if (i === activeIdx) el.classList.add('active');
    else if (i < activeIdx) el.classList.add('done');
  });
  arrows.forEach((a, i) => {
    a.classList.toggle('passed', i < activeIdx);
  });
}

function toggleCam() {
  camOn = !camOn;
  const btn = document.getElementById('camBtn');
  const img = document.getElementById('camImg');
  const off = document.getElementById('camOff');
  if (camOn) {
    btn.textContent = '📷 Disable Camera';
    btn.classList.add('on');
    img.style.display = 'block';
    off.style.display = 'none';
    refreshCam();
    camTimer = setInterval(refreshCam, 200);
  } else {
    btn.textContent = '📷 Enable Camera';
    btn.classList.remove('on');
    img.style.display = 'none';
    img.src = '';
    off.style.display = '';
    if (camTimer) clearInterval(camTimer);
  }
}

function refreshCam() {
  if (!camOn) return;
  document.getElementById('camImg').src = '/camera_feed?' + Date.now();
}

function update() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      document.getElementById('connDot').style.background = '#4caf50';
      document.getElementById('connText').textContent = 'Connected';

      // State
      const sb = document.getElementById('stateBadge');
      const currentState = d.state;
      sb.textContent = currentState;
      sb.className = 'state-badge state-' + currentState;

      // Update competition flow
      updateFlow(currentState);

      // Log state changes
      if (currentState !== lastState && lastState !== '') {
        addLogEntry(`State: <span class="log-val">${lastState}</span> → <span class="log-val">${currentState}</span>`);
      }
      lastState = currentState;

      const mb = document.getElementById('modeBadge');
      const newMode = d.auto_mode ? 'AUTO' : 'MANUAL';
      const oldMode = mb.textContent;
      mb.textContent = newMode;
      mb.className = 'mode-badge mode-' + newMode;
      if (oldMode && oldMode !== newMode) {
        addLogEntry(`Mode: <span class="log-val">${newMode}</span>`);
      }

      document.getElementById('lapBadge').textContent = 'Lap ' + d.lap;
      document.getElementById('stateTime').textContent = d.state_time;
      document.getElementById('stateDist').textContent = d.state_dist;

      // Traffic light
      ['Red','Yellow','Green'].forEach(c => {
        document.getElementById('tl'+c).classList.toggle('active', d.traffic_light === c.toLowerCase());
      });
      document.getElementById('tlText').textContent = d.traffic_light;

      // Sensors
      function ss(dId, vId, v, tl, fl) {
        const dot = document.getElementById(dId), val = document.getElementById(vId);
        if (v===null||v===undefined){dot.className='dot dot-gray';val.textContent='—';return;}
        dot.className = v ? 'dot dot-red' : 'dot dot-green';
        val.textContent = v ? (tl||'YES') : (fl||'NO');
      }
      ss('dotLidar','valLidar',d.lidar_obstacle,'BLOCKED','CLEAR');
      ss('dotCam','valCam',d.camera_obstacle,'BLOCKED','CLEAR');
      ss('dotFused','valFused',d.fused_obstacle,'BLOCKED','CLEAR');
      ss('dotTunnel','valTunnel',d.tunnel_detected,'IN TUNNEL','NO');
      ss('dotObst','valObst',d.obstruction_active,'DODGING','NO');
      ss('dotPark','valPark',d.parking_complete,'DONE','NO');

      const gD=document.getElementById('dotGate'),gV=document.getElementById('valGate');
      if(d.boom_gate===null){gD.className='dot dot-gray';gV.textContent='—';}
      else{gD.className=d.boom_gate?'dot dot-green':'dot dot-red';gV.textContent=d.boom_gate?'OPEN':'CLOSED';}

      // Lane
      document.getElementById('laneErr').textContent = d.lane_error.toFixed(3);
      document.getElementById('laneBar').style.width = Math.min(Math.max((d.lane_error+0.5)/1.0*100,0),100)+'%';
      document.getElementById('cmdLinX').textContent = d.cmd_lin_x.toFixed(3);
      document.getElementById('cmdAngZ').textContent = d.cmd_ang_z.toFixed(3);

      // Odom
      document.getElementById('odomDist').textContent = d.distance.toFixed(2);
      document.getElementById('odomSpeed').textContent = d.speed.toFixed(3)+' m/s';

      // Speed & Selector
      document.getElementById('speedPct').textContent = d.speed_pct+'%';
      document.getElementById('speedBar').style.width = d.speed_pct+'%';
      // Update gear dots
      const gears = [25,40,60,100];
      gears.forEach((g,i) => {
        const dot = document.getElementById('gear'+i);
        if(dot) dot.classList.toggle('active', d.speed_pct >= g);
      });
      document.getElementById('ctrlState').textContent = d.ctrl_state_name;

      // Controller buttons — Updated to match user's specific mapping:
      // A=0, B=1, X=3, Y=4, LB=6, RB=7, Start=11
      const bm={0:'btnA',1:'btnB',3:'btnX',4:'btnY',6:'btnLB',7:'btnRB',8:'btnLT',9:'btnRT',11:'btnStart'};
      Object.values(bm).forEach(id=>document.getElementById(id).classList.remove('active'));
      if(d.buttons){
        d.buttons.forEach((v,i)=>{if(v&&bm[i])document.getElementById(bm[i]).classList.add('active');});
        const p=[];d.buttons.forEach((v,i)=>{if(v)p.push('btn['+i+']');});
        const db=document.getElementById('btnDebug');
        if(p.length){db.textContent='Active: '+p.join(', ');db.style.color='#4caf50';}
        else{db.textContent='No buttons pressed';db.style.color='#333';}
      }

      // D-pad (axes 6,7)
      if(d.axes&&d.axes.length>7){
        document.getElementById('btnLeft').classList.toggle('active',d.axes[6]>0.5);
        document.getElementById('btnRight').classList.toggle('active',d.axes[6]<-0.5);
        document.getElementById('btnUp').classList.toggle('active',d.axes[7]>0.5);
        document.getElementById('btnDown').classList.toggle('active',d.axes[7]<-0.5);
      }
      if(d.axes&&d.axes.length>3){
        document.getElementById('joyL').textContent=d.axes[0].toFixed(1)+', '+d.axes[1].toFixed(1);
        document.getElementById('joyR').textContent=d.axes[2].toFixed(1)+', '+d.axes[3].toFixed(1);
      }
    })
    .catch(()=>{
      document.getElementById('connDot').style.background='#f44336';
      document.getElementById('connText').textContent='Disconnected';
    });
}
// ===== PARAMETER TUNING =====
let paramData = {};  // {node: [{name, value, type}]}
let paramAutoRefresh = null;

async function loadAllParams() {
  document.getElementById('paramContainer').innerHTML = '<div class="param-empty">Loading nodes...</div>';
  try {
    const r = await fetch('/api/params');
    paramData = await r.json();
    renderParams();
  } catch(e) {
    document.getElementById('paramContainer').innerHTML = '<div class="param-empty" style="color:#f44336">Failed to load</div>';
  }
}

async function refreshAllParams() {
  try {
    const r = await fetch('/api/params');
    paramData = await r.json();
    renderParams();
    document.getElementById('paramPoll').textContent = 'Refreshed ' + new Date().toLocaleTimeString('en-GB');
  } catch(e) {}
}

function renderParams() {
  const c = document.getElementById('paramContainer');
  const nodes = Object.keys(paramData);
  if (!nodes.length) {
    c.innerHTML = '<div class="param-empty">No nodes found</div>';
    return;
  }
  c.innerHTML = nodes.map(node => {
    const params = paramData[node] || [];
    const rows = params.map(p =>
      `<div class="param-row" id="pr_${node}_${p.name.replace(/\./g,'_')}">
        <span class="param-name">${p.name}</span>
        <input class="param-val" id="pv_${node}_${p.name.replace(/\./g,'_')}" value="${p.value}" />
        <button class="param-set-btn" onclick="setParam('${node}','${p.name}',this)">Set</button>
        <span class="param-status" id="ps_${node}_${p.name.replace(/\./g,'_')}"></span>
      </div>`
    ).join('');
    return `<div class="param-node-block">
      <div class="param-node-header" onclick="this.nextElementSibling.classList.toggle('open')">
        <span class="param-node-name">${node}</span>
        <span class="param-node-count">${params.length} params</span>
      </div>
      <div class="param-node-body">${rows}</div>
    </div>`;
  }).join('');
}

async function setParam(node, paramName, btn) {
  const inputId = 'pv_' + node + '_' + paramName.replace(/\./g, '_');
  const statusId = 'ps_' + node + '_' + paramName.replace(/\./g, '_');
  const input = document.getElementById(inputId);
  const status = document.getElementById(statusId);
  if (!input) return;
  btn.disabled = true;
  btn.textContent = '...';
  try {
    const r = await fetch('/api/set_param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({node: node, param: paramName, value: input.value})
    });
    const result = await r.json();
    if (result.ok) {
      status.className = 'param-status ok';
      status.textContent = '\u2713 Set';
    } else {
      status.className = 'param-status err';
      status.textContent = result.error || 'Failed';
    }
  } catch(e) {
    status.className = 'param-status err';
    status.textContent = 'Error';
  }
  btn.disabled = false;
  btn.textContent = 'Set';
  setTimeout(() => { if(status) status.textContent = ''; }, 3000);
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

        # CV Bridge for camera
        self.bridge = CvBridge() if CvBridge else None
        self.latest_jpeg = None
        self.jpeg_lock = threading.Lock()

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
        self.create_subscription(String, '/set_challenge', self._set_challenge_cb, 10)
        # Also listen to auto commands for display
        self.create_subscription(Twist, '/cmd_vel_auto', self._cmd_cb, 10)

        # Camera subscription (SENSOR_DATA QoS to match camera publisher)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._image_cb, qos
        )

        self.get_logger().info('Dashboard subscriptions ready')

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

    def _odom_cb(self, msg):
        with self.data_lock:
            self.data['speed'] = msg.twist.twist.linear.x
            # Integrate distance from velocity
            now = time.time()
            if hasattr(self, '_last_odom_t'):
                dt = min(now - self._last_odom_t, 0.2)
                self.data['distance'] += msg.twist.twist.linear.x * dt
            self._last_odom_t = now

    def _joy_cb(self, msg):
        with self.data_lock:
            self.data['buttons'] = list(msg.buttons)
            self.data['axes'] = list(msg.axes)

    def _dash_state_cb(self, msg):
        """Parse STATE|LAP|DIST format from auto_driver."""
        try:
            parts = msg.data.split('|')
            with self.data_lock:
                if parts[0] != self.data.get('state'):
                    self._state_entry_time = time.time()  # Reset timer on state change
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

    def _set_challenge_cb(self, msg):
        """Also listen to /set_challenge as fallback for state cycle display."""
        name = msg.data.upper()
        self._set('ctrl_state_name', name)

    def _image_cb(self, msg):
        """Convert ROS Image to JPEG for the web feed."""
        if self.bridge is None:
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
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(204)
                self.end_headers()
        elif self.path == '/api/params':
            # Discover nodes and their parameters via ros2 param dump
            result = {}
            try:
                nodes_out = subprocess.run(
                    ['ros2', 'node', 'list'],
                    capture_output=True, text=True, timeout=5
                )
                nodes = [n.strip() for n in nodes_out.stdout.strip().split('\n') if n.strip()]
                for node in nodes:
                    short_name = node.lstrip('/')
                    try:
                        # Use ros2 param dump - gets ALL params + values in one call
                        dump_out = subprocess.run(
                            ['ros2', 'param', 'dump', node, '--print'],
                            capture_output=True, text=True, timeout=8
                        )
                        params = []
                        raw = dump_out.stdout.strip()
                        if raw:
                            # Parse YAML-like output:  node_name:\n  ros__parameters:\n    key: value
                            current_key = None
                            for line in raw.split('\n'):
                                stripped = line.strip()
                                if not stripped or stripped.endswith(':') and '.' not in stripped and ':' == stripped[-1]:
                                    # Section header like "node_name:" or "ros__parameters:"
                                    continue
                                if ':' in stripped:
                                    key, _, val = stripped.partition(':')
                                    key = key.strip()
                                    val = val.strip()
                                    if key and val:
                                        params.append({'name': key, 'value': val})
                                    elif key:
                                        # Could be a list/dict - just record the key
                                        current_key = key
                        if not params:
                            # Fallback: use param list only (no values)
                            list_out = subprocess.run(
                                ['ros2', 'param', 'list', node],
                                capture_output=True, text=True, timeout=5
                            )
                            for pname in list_out.stdout.strip().split('\n'):
                                pname = pname.strip()
                                if pname and not pname.startswith('Warning'):
                                    params.append({'name': pname, 'value': '?'})
                        result[short_name] = params
                    except Exception:
                        result[short_name] = []
            except Exception:
                pass
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())

    def do_POST(self):
        if self.path == '/api/set_param':
            content_len = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_len)
            try:
                data = json.loads(body)
                node = data['node']
                param = data['param']
                value = str(data['value'])
                # Ensure node has leading /
                if not node.startswith('/'):
                    node = '/' + node
                out = subprocess.run(
                    ['ros2', 'param', 'set', node, param, value],
                    capture_output=True, text=True, timeout=5
                )
                if out.returncode == 0:
                    resp = {'ok': True, 'msg': out.stdout.strip()}
                else:
                    resp = {'ok': False, 'error': out.stderr.strip() or out.stdout.strip()}
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
