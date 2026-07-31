#!/usr/bin/env bash
# ==============================================================================
# RISA-Bot: Virtual AP Interface Setup (ap0)
# Creates ap0 off the same physical radio as wlan0 for concurrent AP+STA mode.
# Assigns static IP 192.168.4.1/24 and allows traffic from hotspot clients.
# Not a captive portal — clients get real DNS and NAT'd internet via wlan0.
# ==============================================================================

set -e

PHYS_IFACE="wlan0"
VIRT_IFACE="ap0"
AP_IP="192.168.4.1"
NETMASK="24"

log() { echo "[risabot-ap] $*"; }

log "Starting ap0 virtual interface setup..."

# Unblock Wi-Fi if soft-blocked by rfkill
rfkill unblock wifi 2>/dev/null || true

# Discover physical PHY device (e.g. phy0) from wlan0
PHY_DEV=$(iw dev "$PHYS_IFACE" info 2>/dev/null | awk '/wiphy/{print "phy"$2}')
if [ -z "$PHY_DEV" ]; then
    PHY_DEV="phy0"
fi
log "Physical radio: $PHY_DEV"

# Create ap0 if it doesn't exist
if ! ip link show "$VIRT_IFACE" >/dev/null 2>&1; then
    log "Creating virtual interface '$VIRT_IFACE' on $PHY_DEV..."
    iw phy "$PHY_DEV" interface add "$VIRT_IFACE" type managed 2>/dev/null || true
else
    log "Interface '$VIRT_IFACE' already exists, reconfiguring..."
    ip link set "$VIRT_IFACE" down 2>/dev/null || true
fi

# Assign a locally-administered MAC address to avoid conflicts with wlan0
PHYS_MAC=$(cat /sys/class/net/"$PHYS_IFACE"/address 2>/dev/null || echo "02:00:00:00:00:01")
FIRST_BYTE=$(echo "$PHYS_MAC" | cut -d: -f1)
REST_MAC=$(echo "$PHYS_MAC" | cut -d: -f2-)
HEX_VAL=$((0x$FIRST_BYTE ^ 0x02))
NEW_FIRST=$(printf "%02x" $HEX_VAL)
AP_MAC="${NEW_FIRST}:${REST_MAC}"
log "Setting ap0 MAC: $AP_MAC"
ip link set "$VIRT_IFACE" address "$AP_MAC" 2>/dev/null || true

# Assign static IP 192.168.4.1/24
log "Assigning static IP $AP_IP/$NETMASK to $VIRT_IFACE..."
ip addr flush dev "$VIRT_IFACE" 2>/dev/null || true
ip addr add "$AP_IP/$NETMASK" dev "$VIRT_IFACE" broadcast 192.168.4.255
ip link set "$VIRT_IFACE" up

# Tell NetworkManager to leave ap0 unmanaged so it doesn't interfere
nmcli device set "$VIRT_IFACE" managed no 2>/dev/null || true

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward

# Remove the legacy captive-portal REDIRECT if an older install left it behind.
# Hijacking port 80 would break plain HTTP browsing for hotspot clients, who are
# meant to reach the internet through wlan0. The app talks to :8000 directly.
iptables -t nat -D PREROUTING -i "$VIRT_IFACE" -p tcp --dport 80 -j REDIRECT --to-ports 8000 2>/dev/null || true

# iptables: allow traffic from ap0 clients
iptables -D INPUT -i "$VIRT_IFACE" -j ACCEPT 2>/dev/null || true
iptables -A INPUT -i "$VIRT_IFACE" -j ACCEPT

log "ap0 interface is UP at $AP_IP — ready for hostapd."
