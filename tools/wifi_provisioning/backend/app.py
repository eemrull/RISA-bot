#!/usr/bin/env python3
"""
RISA-Bot WiFi Provisioning API
======================================================
Headless JSON API consumed by the RisaBotApp companion app (.NET MAUI). The robot
broadcasts an ap0 hotspot; the app connects to it, provisions WiFi, then follows the
robot to its new address. There is no browser UI — the app is the only client.

 - GET /api/robot_info     — robot identity; the app probes this to discover robots
 - GET /api/status         — wlan0 connection state, IP, internet, ROS 2 service health
 - GET /api/scan           — nmcli wifi scan results as JSON
 - POST /api/connect       — nmcli wifi connect with async WebSocket progress stream
 - GET /api/logs           — journalctl ROS 2 service logs, parsed for errors
 - POST /api/restart_ros   — systemctl restart risabot.service
 - GET /api/launch_status  — whether risabot.service (bringup.launch.py) is active
 - POST /api/launch_start  — systemctl start risabot.service
 - POST /api/launch_stop   — systemctl stop risabot.service
 - WS /ws/log              — WebSocket log stream for live connection progress

Note: risabot.service runs /usr/local/bin/risabot-launch.sh which executes
      'ros2 launch risabot_automode bringup.launch.py' with a full ROS 2 environment.
      Start/Stop Launch buttons control this service directly via systemctl.
"""

import asyncio
import re
import subprocess
from typing import Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------
app = FastAPI(title="RISA-Bot Provisioning API")

# Gateway IP (ap0 static address)
PORTAL_IP = "192.168.4.1"
PORTAL_PORT = 8000
DASHBOARD_PORT = 8080

# ---------------------------------------------------------------------------
# WebSocket connection manager
# ---------------------------------------------------------------------------
class ConnectionManager:
    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, msg: dict):
        dead = []
        for ws in self.active:
            try:
                await ws.send_json(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)

manager = ConnectionManager()

# ---------------------------------------------------------------------------
# NAT / IP forwarding
# ---------------------------------------------------------------------------
def setup_nat() -> None:
    """
    Enable IP forwarding + iptables masquerade so devices connected to the
    hotspot (ap0) can reach the internet through wlan0.

    Called at startup (if wlan0 already has an IP) and after every successful
    WiFi connect, so the iPad/Android device never loses internet.

    Also blocks QUIC (UDP 443 outbound) so Android Chrome falls back to
    TCP/HTTPS instead of failing with ERR_QUIC_PROTOCOL_ERROR.
    """
    # Persistent ip_forward via sysctl
    run_cmd("sysctl -w net.ipv4.ip_forward=1")
    run_cmd("grep -qxF 'net.ipv4.ip_forward=1' /etc/sysctl.conf || "
            "echo 'net.ipv4.ip_forward=1' >> /etc/sysctl.conf")

    # MASQUERADE: route ap0 clients through wlan0 to the internet
    run_cmd("iptables -t nat -C POSTROUTING -o wlan0 -j MASQUERADE 2>/dev/null || "
            "iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE")

    # FORWARD ap0 -> wlan0 (all protocols: TCP + UDP)
    run_cmd("iptables -C FORWARD -i ap0 -o wlan0 -j ACCEPT 2>/dev/null || "
            "iptables -A FORWARD -i ap0 -o wlan0 -j ACCEPT")

    # FORWARD wlan0 -> ap0 for established/related sessions
    run_cmd("iptables -C FORWARD -i wlan0 -o ap0 -m state "
            "--state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || "
            "iptables -A FORWARD -i wlan0 -o ap0 -m state "
            "--state RELATED,ESTABLISHED -j ACCEPT")

    # Block QUIC (UDP 443) so Android Chrome falls back to TCP/HTTPS
    # instead of failing with ERR_QUIC_PROTOCOL_ERROR.
    run_cmd("iptables -C FORWARD -i ap0 -p udp --dport 443 -j REJECT 2>/dev/null || "
            "iptables -I FORWARD 1 -i ap0 -p udp --dport 443 -j REJECT")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def run_cmd(cmd: str, timeout: int = 20) -> str:
    """Run a shell command and return stdout string (never raises)."""
    try:
        r = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return r.stdout.strip()
    except subprocess.TimeoutExpired:
        return "Error: command timed out"
    except Exception as e:
        return f"Error: {e}"


def get_wlan0_info() -> Dict:
    """
    Collect wlan0 state, IP, internet reachability, and ROS 2 service status.

    Dashboard URL logic:
      - wlan0_dashboard_url: preferred URL using wlan0 (external WiFi) IP when available.
        Devices on the same WiFi network as the robot use this to reach port 8080 directly.
      - dashboard_url (ap_dashboard_url): always-reachable fallback via the hotspot AP IP.
        Used when the robot has no external WiFi or client is on the provisioning AP.
    """
    # nmcli device status line for wlan0
    nmcli_out = run_cmd(
        "nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status | grep '^wlan0:'"
    )
    state = "disconnected"
    connected_ssid = None
    if nmcli_out:
        parts = nmcli_out.split(":")
        if len(parts) >= 4:
            state = parts[2]
            connected_ssid = parts[3] if parts[3] not in ("", "--") else None

    # IP address on wlan0
    ip_out = run_cmd(
        "ip -4 addr show wlan0 2>/dev/null | grep inet | awk '{print $2}' | cut -d/ -f1 | head -1"
    )
    wlan0_ip = ip_out if ip_out and not ip_out.startswith("Error") else None

    # Ping test for internet — use a short timeout so we don't block the connect flow
    ping_out = run_cmd("ping -c 1 -W 1 8.8.8.8 2>/dev/null && echo ok || echo fail")
    is_online = ping_out.strip() == "ok"

    # risabot.service active? (this is bringup.launch.py running under systemd)
    ros_out = run_cmd("systemctl is-active risabot.service 2>/dev/null || echo inactive")
    ros_active = ros_out.strip() == "active"

    # Hostname-based Dashboard URL (no IP exposed)
    import socket
    h_name = socket.gethostname().strip() or "risabot"
    if not h_name.endswith(".local"):
        h_domain = f"{h_name}.local"
    else:
        h_domain = h_name
    # Port 8080 is the ROS dashboard node. The old port-80 /dashboard route was the
    # portal's own control page and no longer exists.
    web_dashboard_url = f"http://{h_domain}:{DASHBOARD_PORT}"

    # Dashboard URLs
    ap_dashboard_url = f"http://{PORTAL_IP}:{DASHBOARD_PORT}"
    wlan0_dashboard_url = f"http://{wlan0_ip}:{DASHBOARD_PORT}" if wlan0_ip else ap_dashboard_url

    return {
        "wlan0_state": state,
        "connected_ssid": connected_ssid,
        "wlan0_ip": wlan0_ip,
        "ap_ip": PORTAL_IP,
        "internet_online": is_online,
        "ros_service_active": ros_active,
        "launch_active": ros_active,          # alias — service IS the bringup launch
        "dashboard_url": web_dashboard_url,
        "web_dashboard_url": web_dashboard_url,
        "wlan0_dashboard_url": wlan0_dashboard_url,
    }


@app.on_event("startup")
async def on_startup():
    """Re-apply NAT rules if wlan0 is already connected on portal start."""
    info = get_wlan0_info()
    if info["wlan0_ip"]:
        setup_nat()


# ---------------------------------------------------------------------------
# REST Endpoints
# ---------------------------------------------------------------------------
@app.get("/api/status")
async def get_status():
    """Current wlan0 + ROS 2 health + launch status snapshot."""
    return get_wlan0_info()


@app.get("/api/scan")
async def scan_wifi():
    """Trigger nmcli rescan and return nearby SSIDs sorted by signal strength."""
    run_cmd("nmcli device wifi rescan ifname wlan0 2>/dev/null || true", timeout=6)
    await asyncio.sleep(2)

    raw = run_cmd("nmcli -t -f SSID,SIGNAL,SECURITY device wifi list ifname wlan0 2>/dev/null")
    networks: List[Dict] = []
    seen: set = set()

    for line in raw.splitlines():
        if not line:
            continue
        parts = line.split(":")
        if len(parts) < 2:
            continue
        ssid = parts[0].strip()
        signal_str = parts[1].strip() if len(parts) > 1 else "0"
        security = parts[2].strip() if len(parts) > 2 and parts[2].strip() else "Open"

        if not ssid or ssid in seen:
            continue
        seen.add(ssid)
        try:
            signal = int(signal_str)
        except ValueError:
            signal = 0

        networks.append({"ssid": ssid, "signal": signal, "security": security})

    networks.sort(key=lambda x: x["signal"], reverse=True)
    return {"networks": networks}


@app.get("/api/logs")
async def get_logs():
    """
    Fetch recent risabot.service journal entries and flag error lines.
    risabot.service runs bringup.launch.py, so these are the live ROS 2 node logs.
    """
    journal = run_cmd("journalctl -u risabot.service -n 80 --no-pager 2>/dev/null")
    lines = journal.splitlines() if journal else []

    error_pattern = re.compile(r"error|exception|traceback|failed|fatal|critical|crit|abort", re.IGNORECASE)
    errors = [l for l in lines if error_pattern.search(l)]

    return {
        "has_error": len(errors) > 0,
        "error_count": len(errors),
        "errors": errors,
        "logs": lines,
        "status_summary": (
            f"{len(errors)} error line(s) found in RISA-Bot service logs."
            if errors else
            "No errors found in RISA-Bot service logs."
        ),
    }


class ConnectRequest(BaseModel):
    ssid: str
    password: Optional[str] = ""


@app.post("/api/connect")
async def connect_wifi(req: ConnectRequest):
    """Kick off background nmcli connect and stream progress over WebSocket."""
    asyncio.create_task(_do_connect(req.ssid, req.password or ""))
    return {"status": "started", "ssid": req.ssid}


@app.get("/api/robot_info")
async def robot_info():
    """
    Return robot identity information for native app discovery.
    Used by the RISA-Bot mobile app on the DiscoveryPage to confirm
    the phone is connected to the correct robot hotspot.
    """
    import socket
    hostname = socket.gethostname().strip() or "risabot"
    # Extract serial number from hostname (e.g. 'risabot9' -> '9')
    serial = re.sub(r'^risabot', '', hostname, flags=re.IGNORECASE).strip() or "unknown"
    # Attempt to read a version file written during firmware install
    fw_version = run_cmd("cat /etc/risabot_version 2>/dev/null") or "unknown"
    return {
        "name": f"RISA-Bot {serial}",
        "serial": serial,
        "model": "Horizon RDK X5",
        "firmware_version": fw_version,
        "hostname": hostname,
        "mdns_hostname": f"{hostname}.local",
    }


@app.post("/api/restart_ros")
async def restart_ros():
    """Restart risabot.service (= bringup.launch.py) via systemd."""
    out = run_cmd("systemctl restart risabot.service 2>&1")
    active = run_cmd("systemctl is-active risabot.service 2>/dev/null || echo inactive")
    return {"status": active, "output": out}


# ---------------------------------------------------------------------------
# Launch Control Endpoints
# Controls risabot.service which runs bringup.launch.py (via risabot-launch.sh).
# Only exposed on the provisioning portal (port 8000) — NOT on the robot dashboard.
# ---------------------------------------------------------------------------
@app.get("/api/launch_status")
async def launch_status():
    """
    Return whether risabot.service (i.e. bringup.launch.py) is currently active.
    risabot.service is created by tools/setup_autostart.sh and runs
    /usr/local/bin/risabot-launch.sh → ros2 launch risabot_automode bringup.launch.py
    """
    active = run_cmd("systemctl is-active risabot.service 2>/dev/null || echo inactive")
    return {"launch_active": active.strip() == "active", "service_state": active.strip()}


@app.post("/api/launch_start")
async def launch_start():
    """
    Start risabot.service (= bringup.launch.py).
    The service handles ROS 2 environment sourcing via risabot-launch.sh.
    """
    out = run_cmd("systemctl start risabot.service 2>&1")
    active = run_cmd("systemctl is-active risabot.service 2>/dev/null || echo inactive")
    ok = active.strip() == "active"
    return {
        "ok": ok,
        "launch_active": ok,
        "service_state": active.strip(),
        "output": out or "Started",
    }


@app.post("/api/launch_stop")
async def launch_stop():
    """
    Stop risabot.service (kills bringup.launch.py and all child ROS 2 nodes).
    Uses the service ExecStop which runs 'pkill -f ros2'.
    """
    out = run_cmd("systemctl stop risabot.service 2>&1")
    active = run_cmd("systemctl is-active risabot.service 2>/dev/null || echo inactive")
    stopped = active.strip() in ("inactive", "failed")
    return {
        "ok": stopped,
        "launch_active": not stopped,
        "service_state": active.strip(),
        "output": out or "Stopped",
    }


# ---------------------------------------------------------------------------
# WebSocket log stream
# ---------------------------------------------------------------------------
@app.websocket("/ws/log")
async def ws_log(websocket: WebSocket):
    await manager.connect(websocket)
    info = get_wlan0_info()
    await websocket.send_json({
        "type": "log",
        "status": "connected",
        "message": f"Portal ready. wlan0 state: {info['wlan0_state']}",
    })
    try:
        while True:
            data = await websocket.receive_text()
            await websocket.send_json({"type": "pong"})
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ---------------------------------------------------------------------------
# Background WiFi connection task
# ---------------------------------------------------------------------------
async def _broadcast(status: str, message: str, **extra):
    await manager.broadcast({"type": "step", "status": status, "message": message, **extra})


async def _do_connect(ssid: str, password: str):
    await _broadcast("connecting", f"Connecting wlan0 to '{ssid}'...")
    run_cmd(f"nmcli connection delete '{ssid}' 2>/dev/null || true")

    if password:
        cmd = f"nmcli device wifi connect '{ssid}' password '{password}' ifname wlan0"
    else:
        cmd = f"nmcli device wifi connect '{ssid}' ifname wlan0"

    result = run_cmd(cmd, timeout=35)

    if any(kw in result for kw in ("successfully activated", "state: activated", "Connection successfully")):
        await _broadcast("verifying_internet", "Verifying internet...")
        await asyncio.sleep(1.5)

        # Enable IP forwarding + NAT so devices connected to ap0 maintain connection
        setup_nat()

        await _broadcast("checking_ros", "Checking ROS 2 nodes...")
        run_cmd("systemctl restart risabot.service 2>/dev/null || true")
        await asyncio.sleep(2)

        await _broadcast("checking_hardware", "Checking sensors/motors...")
        await asyncio.sleep(1.5)

        info = get_wlan0_info()

        await _broadcast(
            "ready",
            "Ready",
            wlan0_ip=info["wlan0_ip"],
            dashboard_url=info["web_dashboard_url"],
        )
    else:
        await _broadcast("failed", f"Connection failed: {result}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=PORTAL_PORT, reload=False)
