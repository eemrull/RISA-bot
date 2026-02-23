#!/bin/bash
# ============================================================
# RISA-Bot WiFi Hotspot Setup
# Creates a WiFi hotspot on the robot so your DR can connect
# directly without needing a separate router.
#
# Usage:  sudo bash setup_hotspot.sh [SSID] [PASSWORD]
# Example: sudo bash setup_hotspot.sh RISABOT mypassword123
#
# After running, your DR can:
# 1. Connect to WiFi network "RISABOT" with password "mypassword123"
# 2. Open browser to http://10.42.0.1:8080
# ============================================================

set -e

SSID="${1:-RISABOT}"
PASS="${2:-risabot123}"
IFACE="wlan0"

echo "🤖 RISA-Bot WiFi Hotspot Setup"
echo "================================"
echo "  SSID:     $SSID"
echo "  Password: $PASS"
echo ""

# Install required packages
echo "📦 Installing dependencies..."
apt-get update -qq
apt-get install -y -qq hostapd dnsmasq NetworkManager > /dev/null 2>&1

# Method 1: Try NetworkManager (preferred, works on Ubuntu 22.04+)
if command -v nmcli &> /dev/null; then
    echo "🔧 Setting up via NetworkManager..."
    
    # Delete existing hotspot connection if any
    nmcli connection delete "$SSID" 2>/dev/null || true
    
    # Create hotspot
    nmcli device wifi hotspot ifname "$IFACE" ssid "$SSID" password "$PASS"
    
    # Make it auto-start on boot
    nmcli connection modify "$SSID" connection.autoconnect yes
    nmcli connection modify "$SSID" ipv4.addresses 10.42.0.1/24
    
    echo ""
    echo "✅ Hotspot created successfully!"
    echo ""
    echo "📱 Your DR should:"
    echo "   1. Connect to WiFi: $SSID"
    echo "   2. Password: $PASS"
    echo "   3. Open browser: http://10.42.0.1:8080"
    echo ""
    echo "🔄 The hotspot will auto-start on every boot."
    echo "   To stop:  nmcli connection down '$SSID'"
    echo "   To start: nmcli connection up '$SSID'"
    
else
    echo "❌ NetworkManager not found. Please install it:"
    echo "   sudo apt install network-manager"
    exit 1
fi
