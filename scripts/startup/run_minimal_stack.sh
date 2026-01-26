#!/bin/bash
# Earth Rover - Minimal Stack Launcher
# Starts only essential services for basic operation:
# - MAVROS (Pixhawk)
# - ROSBridge WebSocket
# - Vehicle Control Station Web Server

set -e

# Configuration
EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
VCS_DIR="${EARTH_ROVER_HOME}/vehicle_control_station"

# Pixhawk configuration
PIXHAWK_DEVICE="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0"
PIXHAWK_BAUD="921600"
FCU_URL="${PIXHAWK_DEVICE}:${PIXHAWK_BAUD}"

# Create log directory
mkdir -p "$LOG_DIR"

echo "================================================"
echo "Starting Earth Rover Minimal Stack"
echo "================================================"
echo "Log directory: $LOG_DIR"
echo ""

# 1. MAVROS (Pixhawk Connection)
echo "[1/3] Starting MAVROS..."
nohup ros2 launch mavros px4.launch fcu_url:="$FCU_URL" > "$LOG_DIR/mavros_px4.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/mavros_px4.log)"
sleep 2

# 2. ROSBridge WebSocket Server
echo "[2/3] Starting ROSBridge WebSocket Server..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG_DIR/rosbridge.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/rosbridge.log)"

# 3. Vehicle Control Station (Django Web Server)
echo "[3/3] Starting Vehicle Control Station Web Server..."
cd "$VCS_DIR"
nohup python3 manage.py runserver 0.0.0.0:8000 > "$LOG_DIR/django_vcs.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/django_vcs.log)"

echo ""
echo "================================================"
echo "✓ Minimal Earth Rover stack started"
echo "================================================"
echo ""
echo "Access Vehicle Control Station at:"
echo "  http://localhost:8000"
echo "  http://$(hostname -I | awk '{print $1}'):8000"
echo ""
echo "View logs in: $LOG_DIR"
echo "================================================"

