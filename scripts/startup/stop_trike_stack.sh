#!/bin/bash
# Earth Rover - Stop Trike Stack
# Gracefully stops all ROS nodes and services

set -e

EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
LOG_DIR="${LOG_DIR:-${EARTH_ROVER_HOME}/scripts/logs}"

echo "================================================"
echo "Stopping Earth Rover Trike Stack"
echo "================================================"

# Function to kill process by name pattern
kill_process() {
    local pattern="$1"
    local name="$2"
    local pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    
    if [ -n "$pids" ]; then
        echo "Stopping $name..."
        echo "$pids" | xargs kill -SIGTERM 2>/dev/null || true
        sleep 1
        # Force kill if still running
        pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo "  └─ Force killing $name..."
            echo "$pids" | xargs kill -SIGKILL 2>/dev/null || true
        fi
        echo "  └─ Stopped"
    else
        echo "Skipping $name (not running)"
    fi
}

# Stop all services in reverse order
kill_process "manage.py runserver" "Django VCS Web Server"
kill_process "deepgis_gps_publisher" "DeepGIS GPS Publisher"
kill_process "velodyne" "Velodyne LiDAR"
kill_process "adsb_state_vectors_plot" "ADS-B 2D Plot"
kill_process "adsb_aircraft_state_vectors" "ADS-B State Vectors"
kill_process "rtl_adsb" "RTL ADS-B"
kill_process "web_video_server" "Web Video Server"
kill_process "rosbridge" "ROSBridge WebSocket"
kill_process "realsense" "RealSense Camera"
kill_process "Intensity_Plot" "Spectrometer Intensity Plot"
kill_process "Spectrometer_Data_Publisher" "Spectrometer"
kill_process "launch_grasshopper" "Grasshopper Cameras"
kill_process "mavros" "MAVROS"
kill_process "ros_publish_serial" "Serial Data Publisher"

echo ""
echo "================================================"
echo "✓ Earth Rover stack stopped"
echo "================================================"

# Clean up PID file
rm -f "$LOG_DIR/trike_stack.pids" 2>/dev/null || true

