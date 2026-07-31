#!/usr/bin/env bash
# ==============================================================================
# RISA-Bot WiFi Provisioning — Installer for risabot9
# Installs ap0 interface setup, hostapd, dnsmasq, and the provisioning API backend
# as permanent systemd services (Restart=always).
#
# The API is headless — the RisaBotApp companion app is the only client.
# There is no browser portal and the hotspot is not a captive network.
#
# Run on the robot as root:
#   sudo bash install_wifi_provisioning.sh
# ==============================================================================

set -e

CONF_DIR="/etc/risabot"
PORTAL_DIR="/etc/risabot/portal"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log()  { echo -e "\e[36m[risabot-install]\e[0m $*"; }
ok()   { echo -e "\e[32m[OK]\e[0m $*"; }
fail() { echo -e "\e[31m[ERR]\e[0m $*"; exit 1; }

[[ $EUID -ne 0 ]] && fail "Run as root: sudo bash $0"

log "=== RISA-Bot WiFi Provisioning Installer ==="
log "Script dir: $SCRIPT_DIR"

# ── 1. Install system packages ──────────────────────────────────────────────
log "Installing system dependencies..."
apt-get update -qq
apt-get install -y --no-install-recommends \
    hostapd \
    dnsmasq \
    iw \
    rfkill \
    iptables \
    net-tools \
    python3-pip \
    python3-venv \
    2>/dev/null

ok "System packages installed."

# ── 2. Install Python packages into a venv ──────────────────────────────────
log "Setting up Python venv for the provisioning API..."
python3 -m venv /opt/risabot-portal-venv --system-site-packages
/opt/risabot-portal-venv/bin/pip install --quiet --upgrade pip
/opt/risabot-portal-venv/bin/pip install --quiet \
    "fastapi>=0.110.0" \
    "uvicorn[standard]>=0.29.0" \
    "websockets>=12.0" \
    "pydantic>=2.0"

# Symlink uvicorn into /usr/local/bin for systemd ExecStart
ln -sf /opt/risabot-portal-venv/bin/uvicorn /usr/local/bin/uvicorn
ok "Python venv ready at /opt/risabot-portal-venv"

# ── 3. Stop interfering services ────────────────────────────────────────────
log "Stopping any existing dnsmasq / hostapd instances..."
systemctl stop dnsmasq 2>/dev/null || true
systemctl disable dnsmasq 2>/dev/null || true
systemctl stop hostapd 2>/dev/null || true
systemctl disable hostapd 2>/dev/null || true

# Prevent NetworkManager from managing ap0
log "Configuring NetworkManager to leave ap0 unmanaged..."
cat > /etc/NetworkManager/conf.d/99-risabot-ap0.conf <<'EOF'
[keyfile]
unmanaged-devices=interface-name:ap0
EOF
systemctl reload NetworkManager 2>/dev/null || true
ok "NetworkManager will not touch ap0."

# ── 4. Create config directory and copy files ────────────────────────────────
log "Copying configuration files to $CONF_DIR ..."
mkdir -p "$CONF_DIR"
mkdir -p "$PORTAL_DIR/backend"

# Extract hostname serial (e.g. risabot10 -> 10)
HOSTNAME_FULL=$(hostname 2>/dev/null || echo "risabot10")
ROBOT_NUM=$(echo "$HOSTNAME_FULL" | grep -oP '\d+' || echo "10")

log "Configuring hotspot identity for $HOSTNAME_FULL (Robot $ROBOT_NUM)..."
cp "$SCRIPT_DIR/hostapd.conf"          "$CONF_DIR/hostapd.conf"
sed -i "s|ssid=RISABot-9|ssid=RISABot-${ROBOT_NUM}|g"                   "$CONF_DIR/hostapd.conf"
sed -i "s|wpa_passphrase=sunriserisabot9|wpa_passphrase=sunriserisabot${ROBOT_NUM}|g" "$CONF_DIR/hostapd.conf"

cp "$SCRIPT_DIR/dnsmasq.conf"          "$CONF_DIR/dnsmasq.conf"
cp "$SCRIPT_DIR/backend/app.py"        "$PORTAL_DIR/backend/app.py"
ok "Config files copied and customized for RISABot-${ROBOT_NUM}."

# ── 5. Install ap0 setup script ──────────────────────────────────────────────
log "Installing ap0 setup script to /usr/local/bin/risabot-ap0-setup.sh ..."
cp "$SCRIPT_DIR/ap0_setup.sh" /usr/local/bin/risabot-ap0-setup.sh
chmod +x /usr/local/bin/risabot-ap0-setup.sh
ok "ap0 setup script installed."

# ── 6. Install systemd service units ─────────────────────────────────────────
log "Installing systemd service units..."
SERVICES=(
    "risabot-ap-interface.service"
    "risabot-hostapd.service"
    "risabot-dnsmasq.service"
    "risabot-wifi-portal.service"
)

for svc in "${SERVICES[@]}"; do
    src="$SCRIPT_DIR/systemd/$svc"
    if [ -f "$src" ]; then
        cp "$src" "/etc/systemd/system/$svc"
        ok "  Installed $svc"
    else
        fail "Missing service file: $src"
    fi
done

# ── 7. Reload systemd and enable all services ─────────────────────────────────
log "Enabling and starting all RISA-Bot WiFi provisioning services..."
systemctl daemon-reload

for svc in "${SERVICES[@]}"; do
    systemctl enable "$svc"
    systemctl restart "$svc"
    sleep 1
    STATUS=$(systemctl is-active "$svc" 2>/dev/null || echo "unknown")
    if [ "$STATUS" = "active" ] || [ "$STATUS" = "activating" ]; then
        ok "$svc → $STATUS"
    else
        echo -e "\e[33m[WARN]\e[0m $svc → $STATUS (check: journalctl -u $svc -n 20)"
    fi
done

# ── 8. Verify ap0 interface ───────────────────────────────────────────────────
log "Verifying ap0 interface..."
sleep 2
if ip link show ap0 >/dev/null 2>&1; then
    AP_IP=$(ip -4 addr show ap0 2>/dev/null | grep inet | awk '{print $2}' | head -1)
    ok "ap0 is UP with address: ${AP_IP:-not yet assigned}"
else
    echo -e "\e[33m[WARN]\e[0m ap0 not found yet — check: journalctl -u risabot-ap-interface -n 20"
fi

# ── 9. Summary ────────────────────────────────────────────────────────────────
echo ""
echo -e "\e[36m============================================================\e[0m"
echo -e "\e[1m  RISA-Bot WiFi Provisioning — Installation Complete!\e[0m"
echo -e "\e[36m============================================================\e[0m"
echo ""
echo "  Hotspot SSID    : RISABot-${ROBOT_NUM}"
echo "  Password        : sunriserisabot${ROBOT_NUM}"
echo "  Gateway IP      : 192.168.4.1"
echo "  Provisioning API: http://192.168.4.1:8000/api/robot_info  (RisaBotApp)"
echo "  Dashboard URL   : http://192.168.4.1:8080"
echo ""
echo "  No browser portal — pair the robot using the RisaBotApp companion app."
echo ""
echo "  Services (all Restart=always):"
for svc in "${SERVICES[@]}"; do
    echo "    systemctl status $svc"
done
echo ""
echo "  Live logs:"
echo "    journalctl -u risabot-hostapd -f"
echo "    journalctl -u risabot-wifi-portal -f"
echo -e "\e[36m============================================================\e[0m"
