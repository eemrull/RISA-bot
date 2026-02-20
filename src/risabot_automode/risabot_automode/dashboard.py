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
    font-family: 'Inter', system-ui, -apple-system, sans-serif;
    font-size: 16px; /* Crisper base size for 1440p */
    font-weight: 500;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    overflow-x: hidden;
    position: relative;
  }

  /* ===== ANIMATED ORB BACKGROUND ===== */
  .bg-orbs {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    z-index: -1; overflow: hidden; pointer-events: none;
  }
  .orb {
    position: absolute; border-radius: 50%; filter: blur(60px);
    animation: float 20s infinite ease-in-out alternate; opacity: 0.15;
  }
  .orb-1 { width: 400px; height: 400px; background: rgba(233,69,96,1); top: -10%; left: 10%; animation-delay: 0s; }
  .orb-2 { width: 500px; height: 500px; background: rgba(15,52,96,1); bottom: -20%; right: -10%; animation-delay: -5s; }
  .orb-3 { width: 300px; height: 300px; background: rgba(66,165,245,1); top: 50%; left: 60%; animation-delay: -10s; }
  @keyframes float { 0% { transform: translateY(0) scale(1); } 100% { transform: translateY(-50px) scale(1.1); } }

  /* ===== WARNING FLASH OVERLAY ===== */
  .warning-overlay {
    position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
    box-shadow: inset 0 0 150px rgba(255,0,0,0.6);
    pointer-events: none; opacity: 0; z-index: 999;
    transition: opacity 0.3s;
  }
  .warning-overlay.active { animation: flashDanger 1s infinite alternate; }
  @keyframes flashDanger { 0% { opacity: 0.2; } 100% { opacity: 0.7; } }

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
  }
  .col { display: flex; flex-direction: column; gap: 12px; }

  /* ===== CARDS ===== */
  .card {
    background: rgba(18,18,35,0.65);
    backdrop-filter: blur(20px);
    -webkit-backdrop-filter: blur(20px);
    border: 1px solid rgba(255,255,255,0.08);
    border-radius: var(--radius);
    padding: 18px;
    transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
    position: relative;
    overflow: hidden;
    box-shadow: 0 10px 30px rgba(0,0,0,0.2);
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
    border-color: rgba(233,69,96,0.3);
    box-shadow: 0 15px 35px rgba(233,69,96,0.15), inset 0 0 20px rgba(255,255,255,0.02);
    transform: translateY(-4px) scale(1.02);
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
    padding: 2px 8px;
    border-radius: 4px;
    font-weight: 700;
    font-size: 0.6em;
    letter-spacing: 1px;
    transition: all 0.3s;
    vertical-align: middle;
  }
  .mode-AUTO { background: #4caf50; color: #fff; box-shadow: 0 0 8px rgba(76,175,80,0.3); }
  .mode-MANUAL { background: #ff9800; color: #fff; box-shadow: 0 0 8px rgba(255,152,0,0.3); }

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
    display: flex; gap: 12px; align-items: center; justify-content: center;
    background: linear-gradient(145deg, #111, #080808);
    padding: 14px 24px; border-radius: 40px;
    width: fit-content; margin: 0 auto;
    box-shadow: inset 0 4px 10px rgba(0,0,0,0.8), 0 2px 8px rgba(255,255,255,0.05);
    border: 1px solid rgba(255,255,255,0.05);
  }
  .tl-circle {
    width: 32px; height: 32px; border-radius: 50%;
    border: 2px solid #000; transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
    box-shadow: inset 0 2px 6px rgba(0,0,0,0.8);
  }
  .tl-red { background: #3a1111; }
  .tl-yellow { background: #3a3311; }
  .tl-green { background: #113a11; }

  .tl-red.active { 
    background: radial-gradient(circle at 30% 30%, #ff8a80, #f44336); 
    box-shadow: 0 0 20px #f44336, 0 0 40px rgba(244,67,54,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ff5252;
  }
  .tl-yellow.active { 
    background: radial-gradient(circle at 30% 30%, #fff59d, #ffeb3b); 
    box-shadow: 0 0 20px #ffeb3b, 0 0 40px rgba(255,235,59,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #ffff00;
  }
  .tl-green.active { 
    background: radial-gradient(circle at 30% 30%, #a5d6a7, #4caf50); 
    box-shadow: 0 0 20px #4caf50, 0 0 40px rgba(76,175,80,0.4), inset 0 -2px 6px rgba(0,0,0,0.3);
    border-color: #69f0ae;
  }
  .tl-label { text-align: center; margin-top: 8px; font-size: 0.75em; color: var(--muted); font-weight: 600; text-transform: uppercase; letter-spacing: 1px; }

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
    width: 12px; height: 12px; border-radius: 50%; background: rgba(255,255,255,0.08);
    border: 1px solid rgba(255,255,255,0.1);
    transition: all 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275);
  }
  .gear-dot.active {
    background: var(--accent); border-color: #fff;
    box-shadow: 0 0 15px rgba(233,69,96,0.8), 0 0 30px rgba(233,69,96,0.4);
    transform: scale(1.3);
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

  /* ===== PARAMETER DRAWER ===== */
  .param-popout-tab {
    position: fixed; left: 0; top: 50%; transform: translateY(-50%);
    background: rgba(18,18,35,0.95); backdrop-filter: blur(16px);
    padding: 10px 8px; border-radius: 0 10px 10px 0;
    border: 1px solid var(--card-border); border-left: none;
    cursor: pointer; z-index: 200; writing-mode: vertical-lr;
    text-orientation: mixed; font-size: 0.7em; font-weight: 600;
    color: #42a5f5; letter-spacing: 1px; transition: all 0.3s;
  }
  .param-popout-tab:hover { background: rgba(66,165,245,0.15); padding-left: 12px; }
  .param-drawer {
    position: fixed; left: -500px; top: 60px; bottom: 0; width: 440px; /* Widened for 1440p */
    background: rgba(10,10,25,0.97); backdrop-filter: blur(20px);
    border-right: 1px solid rgba(66,165,245,0.15); z-index: 199;
    transition: left 0.4s cubic-bezier(0.4,0,0.2,1); overflow-y: auto;
    padding: 20px; box-shadow: 4px 0 30px rgba(0,0,0,0.5);
  }
  .param-drawer.open { left: 0; }
  .param-drawer h3 {
    font-size: 0.85em; text-transform: uppercase; letter-spacing: 2px;
    color: #42a5f5; margin-bottom: 8px; font-weight: 700;
  }
  .param-drawer .note { font-size: 0.8em; color: var(--muted); margin-bottom: 12px; }
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
    display: grid;
    grid-template-columns: 1fr 100px 42px 42px;
    align-items: center;
    padding: 6px 14px;
    border-top: 1px solid rgba(255,255,255,0.03);
    gap: 8px;
    position: relative;
  }
  .param-name {
    font-size: 0.85em;
    color: #e0e0e0;
    font-weight: 500;
  }
  .param-val {
    width: 100%;
    box-sizing: border-box;
    padding: 6px 8px;
    border-radius: 4px;
    border: 1px solid rgba(255,255,255,0.15);
    background: rgba(0,0,0,0.4);
    color: #fff;
    font-size: 0.8em;
    font-family: 'Segoe UI', Roboto, sans-serif;
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
    position: absolute;
    right: 14px;
    bottom: -8px;
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

<!-- BACKGROUND ORBS -->
<div class="bg-orbs">
  <div class="orb orb-1"></div>
  <div class="orb orb-2"></div>
  <div class="orb orb-3"></div>
</div>

<!-- WARNING OVERLAY -->
<div id="warning-overlay" class="warning-overlay"></div>

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
      <h3 style="display:flex;align-items:center;justify-content:space-between;">State Machine <span class="mode-badge" id="modeBadge">MANUAL</span></h3>
      <div style="margin-top:4px;">
        <span class="state-badge" id="stateBadge">—</span>
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
        <div class="num" id="speedPct">25%</div>
        <div class="unit">Drive Speed</div>
        <div class="gear-dots">
          <div class="gear-dot active" id="gear0"></div>
          <div class="gear-dot" id="gear1"></div>
          <div class="gear-dot" id="gear2"></div>
          <div class="gear-dot" id="gear3"></div>
        </div>
      </div>
      <div class="meter"><div class="meter-fill meter-orange" id="speedBar" style="width:25%"></div></div>
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
<!-- ===== PARAMETER TUNING DRAWER ===== -->
<div class="param-popout-tab" onclick="toggleParamDrawer()">⚙️ Parameters</div>
<div class="param-drawer" id="paramDrawer">
  <h3>⚙️ Parameter Tuning</h3>
  <div class="note">💡 Changes apply instantly to nodes but revert to defaults upon restart.</div>
  <div id="paramContainer"></div>
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

function toggleParamDrawer() {
  document.getElementById('paramDrawer').classList.toggle('open');
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

      // Warning Flash Overlay (if blocked in AUTO)
      const isBlocked = d.fused_obstacle; // Use fused for reliable alarm
      const overlay = document.getElementById('warning-overlay');
      if (overlay) {
        if (d.auto_mode && isBlocked) overlay.classList.add('active');
        else overlay.classList.remove('active');
      }

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
// ===== PARAMETER TUNING (curated) =====
const PARAM_GROUPS = [
  { node: 'traffic_light_detector', label: '🚦 Traffic Light', params: [
    'red_h_low1','red_h_high1','red_h_low2','red_h_high2',
    'yellow_h_low','yellow_h_high','green_h_low','green_h_high',
    'sat_min','val_min','min_circle_radius','max_circle_radius','min_pixel_count'
  ]},
  { node: 'line_follower_camera', label: '📐 Line Follower', params: [
    'smoothing_alpha','dead_zone','white_threshold','crop_ratio','show_debug'
  ]},
  { node: 'auto_driver', label: '🚗 Auto Driver', params: [
    'steering_gain','forward_speed','stale_timeout',
    'dist_obstruction_clear','dist_roundabout','dist_boom_gate_1_pass',
    'dist_boom_gate_2_pass','dist_hill','dist_bumper',
    'dist_traffic_light_pass','dist_drive_to_perp'
  ]},
  { node: 'boom_gate_detector', label: '🚧 Boom Gate', params: [
    'min_detect_dist','max_detect_dist','angle_window',
    'min_gate_points','distance_variance_max','lidar_angle_offset'
  ]},
  { node: 'obstruction_avoidance', label: '🔀 Obstruction', params: [
    'detect_dist','clear_dist','front_angle','side_angle_min','side_angle_max',
    'steer_speed','steer_angular','pass_speed','pass_duration',
    'steer_back_duration','lidar_angle_offset'
  ]},
  { node: 'parking_controller', label: '🅿️ Parking', params: [
    'parallel_forward_dist','parallel_reverse_dist','parallel_steer_angle',
    'perp_turn_angle','perp_forward_dist','park_wait_time',
    'drive_speed','reverse_speed'
  ]},
  { node: 'obstacle_avoidance_camera', label: '📷 Camera Obstacle', params: [
    'white_threshold','hysteresis_on','hysteresis_off'
  ]},
];

function buildParamUI() {
  const c = document.getElementById('paramContainer');
  c.innerHTML = PARAM_GROUPS.map(g => {
    const rows = g.params.map(p => 
      `<div class="param-row">
        <span class="param-name">${p}</span>
        <input class="param-val" id="pv_${g.node}_${p}" placeholder="—" />
        <button class="param-set-btn" style="background:rgba(66,165,245,0.1);border-color:rgba(66,165,245,0.3);color:#64b5f6;" onclick="getParam('${g.node}','${p}')">Get</button>
        <button class="param-set-btn" onclick="setParam('${g.node}','${p}')">Set</button>
        <span class="param-status" id="ps_${g.node}_${p}"></span>
      </div>`
    ).join('');
    return `<div class="param-node-block">
      <div class="param-node-header" onclick="this.nextElementSibling.classList.toggle('open')">
        <span class="param-node-name">${g.label}</span>
        <span class="param-node-count">${g.params.length} params</span>
      </div>
      <div class="param-node-body">${rows}</div>
    </div>`;
  }).join('');
}

async function getParam(node, param, isInitialLoad=false) {
  const status = document.getElementById('ps_' + node + '_' + param);
  const input = document.getElementById('pv_' + node + '_' + param);
  if (status && !isInitialLoad) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/get_param?node=' + encodeURIComponent(node) + '&param=' + encodeURIComponent(param));
    const d = await r.json();
    if (d.ok) {
      if (input) {
        input.value = d.value;
        if (isInitialLoad) {
          input.placeholder = `Def: ${d.value}`;
          const pName = input.parentElement.querySelector('.param-name');
          if (pName) {
            pName.innerHTML = `${param} <span style="color:#555;font-size:0.85em;">(def: ${d.value})</span>`;
          }
        }
      }
      if (status && !isInitialLoad) { status.className = 'param-status ok'; status.textContent = '✓'; }
    } else {
      if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = d.error || 'Not found'; }
    }
  } catch(e) {
    if (status && !isInitialLoad) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  if (!isInitialLoad) {
    setTimeout(() => { if(status) status.textContent = ''; }, 3000);
  }
}

async function setParam(node, param) {
  const input = document.getElementById('pv_' + node + '_' + param);
  const status = document.getElementById('ps_' + node + '_' + param);
  if (!input || !input.value) { if(status){status.className='param-status err';status.textContent='Empty';} return; }
  if (status) { status.className = 'param-status'; status.textContent = '...'; }
  try {
    const r = await fetch('/api/set_param', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({node: node, param: param, value: input.value})
    });
    const d = await r.json();
    if (d.ok) {
      if (status) { status.className = 'param-status ok'; status.textContent = '✓ Set'; }
    } else {
      if (status) { status.className = 'param-status err'; status.textContent = d.error || 'Failed'; }
    }
  } catch(e) {
    if (status) { status.className = 'param-status err'; status.textContent = 'Error'; }
  }
  setTimeout(() => { if(status) status.textContent = ''; }, 3000);
}

buildParamUI();

// Auto-fetch parameters on load to populate current values and defaults
setTimeout(async () => {
  for (const g of PARAM_GROUPS) {
    for (const p of g.params) {
      await getParam(g.node, p, true);
    }
  }
}, 1000);

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
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(204)
                self.end_headers()
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
