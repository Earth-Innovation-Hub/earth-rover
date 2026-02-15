#!/bin/bash
# Earth Rover - Full Trike Stack Launcher
# Starts all ROS nodes and services for the complete rover system
#
# Launches: MAVROS, cameras (Grasshopper3, RealSense), spectrometer (data + intensity plot),
# ROSBridge, web video server, Velodyne, DeepGIS GPS, RTL ADS-B, aircraft state vectors,
# ADS-B 2D plot image, Django VCS.

set -e

# Load project .env if present (GCS_URL, PIXHAWK_DEVICE, etc.)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
if [ -f "$EARTH_ROVER_HOME/vehicle_control_station/.env" ]; then
    set -a
    # shellcheck source=/dev/null
    source "$EARTH_ROVER_HOME/vehicle_control_station/.env"
    set +a
fi

# Configuration
EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
ROS2_WS="${ROS2_WS:-/home/jdas/ros2_ws}"
VCS_DIR="${EARTH_ROVER_HOME}/vehicle_control_station"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# Pixhawk configuration (override FCU_URL or PIXHAWK_DEVICE/PIXHAWK_BAUD)
PIXHAWK_DEVICE="${PIXHAWK_DEVICE:-/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0}"
PIXHAWK_BAUD="${PIXHAWK_BAUD:-921600}"
FCU_URL="${FCU_URL:-${PIXHAWK_DEVICE}:${PIXHAWK_BAUD}}"
# GCS URL for QGroundControl etc. (set empty to omit). Your history: udp://192.168.1.7:14550
GCS_URL="${GCS_URL:-udp://192.168.1.7:14550}"

# Source ROS and workspace so deepgis_vehicles and spectrometery_ros2 are available
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
fi

# Create log directory
mkdir -p "$LOG_DIR"

echo "================================================"
echo "Starting Earth Rover Trike Stack"
echo "================================================"
echo "Log directory: $LOG_DIR"
echo ""

# Change to home directory for compatibility
cd /home/jdas

# 1. Serial Data Publisher
echo "[1/14] Starting Serial Data Publisher..."
if [ -f ros_publish_serial.py ]; then
    nohup python3 ros_publish_serial.py > "$LOG_DIR/ros_publish_serial.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/ros_publish_serial.log)"
else
    echo "  └─ SKIP: ros_publish_serial.py not found"
fi

# 2. MAVROS (Pixhawk Connection) — same as your history: fcu_url + gcs_url
echo "[2/14] Starting MAVROS..."
if [ -n "$GCS_URL" ]; then
    nohup ros2 launch mavros px4.launch fcu_url:="$FCU_URL" gcs_url:="$GCS_URL" > "$LOG_DIR/mavros_px4.log" 2>&1 &
else
    nohup ros2 launch mavros px4.launch fcu_url:="$FCU_URL" > "$LOG_DIR/mavros_px4.log" 2>&1 &
fi
echo "  └─ PID: $! (log: $LOG_DIR/mavros_px4.log)"
sleep 2  # Give MAVROS time to connect

# 3. Grasshopper3 Stereo Cameras
echo "[3/14] Starting Grasshopper3 Stereo Cameras..."
GRASSHOPPER_LAUNCH="${GRASSHOPPER_LAUNCH:-$ROS2_WS/launch_grasshopper.sh}"
if [ -f "$GRASSHOPPER_LAUNCH" ]; then
    nohup "$GRASSHOPPER_LAUNCH" 3 > "$LOG_DIR/grasshopper_stereo.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/grasshopper_stereo.log)"
else
    echo "  └─ SKIP: launch_grasshopper.sh not found (GRASSHOPPER_LAUNCH=$GRASSHOPPER_LAUNCH)"
fi

# 4. Spectrometer Data Publisher (source: spectrometer_ocean_optics, installs as spectrometery_ros2)
echo "[4/14] Starting Spectrometer Data Publisher..."
SPECTROMETER_PY=""
for p in "$ROS2_WS/src/spectrometer_ocean_optics/src/Spectrometer_Data_Publisher.py" \
         "$ROS2_WS/src/spectrometery_ros2/src/Spectrometer_Data_Publisher.py"; do
    if [ -f "$p" ]; then
        SPECTROMETER_PY="$p"
        break
    fi
done
if [ -n "$SPECTROMETER_PY" ]; then
    nohup python3 "$SPECTROMETER_PY" > "$LOG_DIR/spectrometer.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/spectrometer.log)"
else
    echo "  └─ SKIP: Spectrometer_Data_Publisher.py not found"
fi

# 5. Spectrometer Intensity Plot (image topic for VCS; no GUI)
echo "[5/14] Starting Spectrometer Intensity Plot..."
if ros2 pkg list 2>/dev/null | grep -q spectrometery_ros2; then
    nohup ros2 run spectrometery_ros2 Intensity_Plot.py > "$LOG_DIR/spectrometer_intensity_plot.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/spectrometer_intensity_plot.log)"
else
    echo "  └─ SKIP: spectrometery_ros2 not found (source ros2_ws/install/setup.bash?)"
fi

# 6. RealSense Camera
echo "[6/14] Starting RealSense Camera..."
nohup ros2 launch realsense2_camera rs_launch.py > "$LOG_DIR/realsense.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/realsense.log)"

# 7. ROSBridge WebSocket Server
echo "[7/14] Starting ROSBridge WebSocket Server..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG_DIR/rosbridge.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/rosbridge.log)"

# 8. Web Video Server
echo "[8/14] Starting Web Video Server (port 8080)..."
nohup ros2 run web_video_server web_video_server --ros-args -p port:=8080 > "$LOG_DIR/web_video_server.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/web_video_server.log)"

# 9. RTL ADS-B (dump1090; HTTP on 8082 to avoid 8080)
echo "[9/14] Starting RTL ADS-B decoder..."
if ros2 pkg list 2>/dev/null | grep -q deepgis_vehicles; then
    nohup ros2 launch deepgis_vehicles rtl_adsb.launch.py > "$LOG_DIR/rtl_adsb.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/rtl_adsb.log)"
    sleep 3
else
    echo "  └─ SKIP: deepgis_vehicles not found"
fi

# 10. ADS-B aircraft state vectors (trike frame; needs MAVROS + RTL ADS-B)
echo "[10/14] Starting ADS-B aircraft state vectors..."
if ros2 pkg list 2>/dev/null | grep -q deepgis_vehicles; then
    nohup ros2 launch deepgis_vehicles adsb_aircraft_state_vectors.launch.py > "$LOG_DIR/adsb_state_vectors.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/adsb_state_vectors.log)"
    sleep 2
else
    echo "  └─ SKIP: deepgis_vehicles not found"
fi

# 11. ADS-B 2D plot image (publishes ~/aircraft_plot_image for VCS)
echo "[11/14] Starting ADS-B 2D plot image..."
if ros2 pkg list 2>/dev/null | grep -q deepgis_vehicles; then
    nohup ros2 launch deepgis_vehicles adsb_state_vectors_plot_2d.launch.py > "$LOG_DIR/adsb_plot_2d.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/adsb_plot_2d.log)"
else
    echo "  └─ SKIP: deepgis_vehicles not found"
fi

# 12. Velodyne LiDAR
echo "[12/14] Starting Velodyne VLP-16 LiDAR..."
nohup ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py > "$LOG_DIR/velodyne.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/velodyne.log)"

# 13. DeepGIS GPS Publisher
echo "[13/14] Starting DeepGIS GPS Publisher..."
nohup ros2 run deepgis_vehicles deepgis_gps_publisher.py > "$LOG_DIR/deepgis_gps.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/deepgis_gps.log)"

# 14. Vehicle Control Station (Django Web Server)
echo "[14/14] Starting Vehicle Control Station Web Server..."
cd "$VCS_DIR"
nohup python3 manage.py runserver 0.0.0.0:8000 > "$LOG_DIR/django_vcs.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/django_vcs.log)"

echo ""
echo "================================================"
echo "✓ All Earth Rover services started"
echo "================================================"
echo ""
echo "Access Vehicle Control Station at:"
echo "  http://localhost:8000"
echo "  http://$(hostname -I | awk '{print $1}'):8000"
echo ""
echo "View logs in: $LOG_DIR"
echo ""
echo "To stop all services, run:"
echo "  $EARTH_ROVER_HOME/scripts/startup/stop_trike_stack.sh"
echo "================================================"

# Save PIDs to file for easier shutdown
pgrep -f "ros_publish_serial.py|mavros|grasshopper|Spectrometer|Intensity_Plot|realsense|rosbridge|web_video_server|rtl_adsb|adsb_aircraft_state_vectors|adsb_state_vectors_plot|velodyne|deepgis_gps|manage.py runserver" \
    > "$LOG_DIR/trike_stack.pids" 2>/dev/null || true

