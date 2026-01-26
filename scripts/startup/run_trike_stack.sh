#!/bin/bash
# Earth Rover - Full Trike Stack Launcher
# Starts all ROS nodes and services for the complete rover system
#
# This script launches:
# - MAVROS (Pixhawk connection)
# - Camera nodes (Grasshopper3 stereo, RealSense)
# - Sensor nodes (Spectrometer, Serial data, Velodyne LiDAR)
# - Web services (ROSBridge, Web Video Server, Django VCS)
# - DeepGIS GPS publisher

set -e

# Configuration
EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"
ROS2_WS="/home/jdas/ros2_ws"
VCS_DIR="${EARTH_ROVER_HOME}/vehicle_control_station"

# Pixhawk configuration
PIXHAWK_DEVICE="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0"
PIXHAWK_BAUD="921600"
FCU_URL="${PIXHAWK_DEVICE}:${PIXHAWK_BAUD}"

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
echo "[1/10] Starting Serial Data Publisher..."
if [ -f ros_publish_serial.py ]; then
    nohup python3 ros_publish_serial.py > "$LOG_DIR/ros_publish_serial.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/ros_publish_serial.log)"
else
    echo "  └─ SKIP: ros_publish_serial.py not found"
fi

# 2. MAVROS (Pixhawk Connection)
echo "[2/10] Starting MAVROS..."
nohup ros2 launch mavros px4.launch fcu_url:="$FCU_URL" > "$LOG_DIR/mavros_px4.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/mavros_px4.log)"
sleep 2  # Give MAVROS time to connect

# 3. Grasshopper3 Stereo Cameras
echo "[3/10] Starting Grasshopper3 Stereo Cameras..."
if [ -f /home/amal/ros2_ws/launch_grasshopper.sh ]; then
    nohup /home/amal/ros2_ws/launch_grasshopper.sh 3 > "$LOG_DIR/grasshopper_stereo.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/grasshopper_stereo.log)"
else
    echo "  └─ SKIP: launch_grasshopper.sh not found"
fi

# 4. Spectrometer Data Publisher
echo "[4/10] Starting Spectrometer Data Publisher..."
if [ -f "$ROS2_WS/src/spectrometery_ros2/src/Spectrometer_Data_Publisher.py" ]; then
    nohup python3 "$ROS2_WS/src/spectrometery_ros2/src/Spectrometer_Data_Publisher.py" \
        > "$LOG_DIR/spectrometer.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/spectrometer.log)"
else
    echo "  └─ SKIP: Spectrometer_Data_Publisher.py not found"
fi

# 5. RealSense Camera
echo "[5/10] Starting RealSense Camera..."
nohup ros2 launch realsense2_camera rs_launch.py > "$LOG_DIR/realsense.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/realsense.log)"

# 6. ROSBridge WebSocket Server
echo "[6/10] Starting ROSBridge WebSocket Server..."
nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG_DIR/rosbridge.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/rosbridge.log)"

# 7. Web Video Server
echo "[7/10] Starting Web Video Server..."
nohup ros2 run web_video_server web_video_server > "$LOG_DIR/web_video_server.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/web_video_server.log)"

# 8. Velodyne LiDAR
echo "[8/10] Starting Velodyne VLP-16 LiDAR..."
nohup ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py > "$LOG_DIR/velodyne.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/velodyne.log)"

# 9. DeepGIS GPS Publisher
echo "[9/10] Starting DeepGIS GPS Publisher..."
nohup ros2 run deepgis_vehicles deepgis_gps_publisher.py > "$LOG_DIR/deepgis_gps.log" 2>&1 &
echo "  └─ PID: $! (log: $LOG_DIR/deepgis_gps.log)"

# 10. Vehicle Control Station (Django Web Server)
echo "[10/10] Starting Vehicle Control Station Web Server..."
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
pgrep -f "ros_publish_serial.py|mavros|grasshopper|Spectrometer|realsense|rosbridge|web_video_server|velodyne|deepgis_gps|manage.py runserver" \
    > "$LOG_DIR/trike_stack.pids" 2>/dev/null || true

