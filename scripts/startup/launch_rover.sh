#!/bin/bash
################################################################################
# Earth Rover - Unified Launch Script
# 
# Single script to launch Earth Rover in different configurations.
# All ROS2 services now use the unified earth_rover.launch.py file.
#
# SHELL MODES (multi-process, background):
#   - minimal:           MAVROS + ROSBridge + VCS Web Server
#   - trike:             Full trike stack with all sensors
#
# ROS2 MODES (single-process, foreground):
#   - vehicle_interface: MAVROS + Vehicle Interface only
#   - telemetry:         MAVROS + Vehicle Interface + DeepGIS Telemetry
#   - full_system:       All ROS2 services (MAVROS + Telemetry + Web)
#   - custom:            Custom configuration using enable flags
#
# Usage: 
#   ./launch_rover.sh [MODE] [options]
#
# Examples:
#   ./launch_rover.sh minimal
#   ./launch_rover.sh trike
#   ROS_STARTUP_DELAY=15 ./launch_rover.sh trike
#   ./launch_rover.sh telemetry deepgis_api_url:=http://192.168.0.186:8080
#   ./launch_rover.sh custom enable_vehicle:=true enable_web:=true
################################################################################

set -e

# ============================================================================
# Configuration
# ============================================================================

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Environment variables with defaults
export EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export LOG_DIR="${EARTH_ROVER_HOME}/scripts/logs"
export ROS2_WS="/home/jdas/ros2_ws"
export VCS_DIR="${EARTH_ROVER_HOME}/vehicle_control_station"

# Pixhawk configuration
PIXHAWK_DEVICE="/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0"
PIXHAWK_BAUD="921600"
FCU_URL="${PIXHAWK_DEVICE}:${PIXHAWK_BAUD}"

# Create log directory
mkdir -p "$LOG_DIR"

# Target mode
TARGET="${1:-trike}"

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║          Earth Rover - Unified Launch Script              ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Configuration:"
    echo "  Rover Home:   $EARTH_ROVER_HOME"
    echo "  ROS Distro:   $ROS_DISTRO"
    echo "  Log Dir:      $LOG_DIR"
    echo "  Target:       $TARGET"
    echo ""
}

setup_ros_environment() {
    echo "→ Sourcing ROS 2 $ROS_DISTRO environment..."
    source /opt/ros/$ROS_DISTRO/setup.bash
    
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        echo "→ Sourcing ROS 2 workspace..."
        source $ROS2_WS/install/setup.bash
    fi
}

apply_startup_delay() {
    if [[ -n "$ROS_STARTUP_DELAY" && "$ROS_STARTUP_DELAY" =~ ^[0-9]+$ && "$ROS_STARTUP_DELAY" -gt 0 ]]; then
        echo "→ Waiting ${ROS_STARTUP_DELAY}s for USB/devices..."
        sleep "$ROS_STARTUP_DELAY"
    fi
}

start_service() {
    local step="$1"
    local name="$2"
    local command="$3"
    local logfile="$4"
    
    echo "[$step] Starting $name..."
    nohup bash -c "$command" > "$logfile" 2>&1 &
    echo "  └─ PID: $! (log: $logfile)"
}

start_ros2_service() {
    local step="$1"
    local name="$2"
    local ros_command="$3"
    local logfile="$4"
    
    start_service "$step" "$name" \
        "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS2_WS/install/setup.bash && $ros_command" \
        "$logfile"
}

start_rosbridge() {
    local step="$1"
    start_service "$step" "ROSBridge WebSocket Server" \
        "source /opt/ros/$ROS_DISTRO/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" \
        "$LOG_DIR/rosbridge.log"
}

start_django_vcs() {
    local step="$1"
    echo "[$step] Starting Vehicle Control Station Web Server..."
    cd "$VCS_DIR"
    export ROS_BRIDGE_URL="${ROS_BRIDGE_URL:-ws://192.168.1.7:9090}"
    export VIDEO_SERVER_URL="${VIDEO_SERVER_URL:-http://192.168.1.7:8080}"
    nohup python3 manage.py runserver 0.0.0.0:8000 > "$LOG_DIR/django_vcs.log" 2>&1 &
    echo "  └─ PID: $! (log: $LOG_DIR/django_vcs.log)"
}

# ============================================================================
# Launch Modes
# ============================================================================

launch_minimal() {
    echo "════════════════════════════════════════════════════════════"
    echo "Starting Earth Rover - MINIMAL Stack"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    
    # Use unified ROS2 launch file
    start_ros2_service "1/3" "MAVROS + Vehicle Interface" \
        "ros2 launch $EARTH_ROVER_HOME/launch/earth_rover.launch.py mode:=minimal fcu_url:=$FCU_URL" \
        "$LOG_DIR/earth_rover.log"
    
    sleep 3  # Give MAVROS time to connect
    
    # ROSBridge WebSocket Server
    start_rosbridge "2/3"
    
    # Vehicle Control Station (Django Web Server)
    start_django_vcs "3/3"
    
    print_minimal_footer
}

launch_trike() {
    echo "════════════════════════════════════════════════════════════"
    echo "Starting Earth Rover - FULL TRIKE Stack"
    echo "════════════════════════════════════════════════════════════"
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
    
    # 2. MAVROS + Vehicle Interface (using unified ROS2 launch file)
    start_ros2_service "2/10" "MAVROS + Vehicle Interface" \
        "ros2 launch $EARTH_ROVER_HOME/launch/earth_rover.launch.py mode:=minimal fcu_url:=$FCU_URL" \
        "$LOG_DIR/earth_rover.log"
    
    sleep 3  # Give MAVROS time to connect
    
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
    start_ros2_service "5/10" "RealSense Camera" \
        "ros2 launch realsense2_camera rs_launch.py" \
        "$LOG_DIR/realsense.log"
    
    # 6. ROSBridge WebSocket Server
    start_rosbridge "6/10"
    
    # 7. Web Video Server
    start_ros2_service "7/10" "Web Video Server (port ${WEB_VIDEO_PORT:-8080})" \
        "ros2 run web_video_server web_video_server --ros-args -p port:=${WEB_VIDEO_PORT:-8080}" \
        "$LOG_DIR/web_video_server.log"
    
    # 8. Velodyne LiDAR
    start_ros2_service "8/10" "Velodyne VLP-16 LiDAR" \
        "ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py" \
        "$LOG_DIR/velodyne.log"
    
    # 9. DeepGIS GPS Publisher (using ROS2 package, not standalone script)
    start_ros2_service "9/10" "DeepGIS GPS Publisher" \
        "ros2 run deepgis_vehicles deepgis_gps_publisher.py" \
        "$LOG_DIR/deepgis_gps.log"
    
    # 10. Vehicle Control Station (Django Web Server)
    start_django_vcs "10/10"
    
    # Save PIDs to file for easier shutdown
    pgrep -f "ros_publish_serial.py|earth_rover.launch|grasshopper|Spectrometer|realsense|rosbridge|web_video_server|velodyne|deepgis_gps|manage.py runserver" \
        > "$LOG_DIR/trike_stack.pids" 2>/dev/null || true
    
    print_trike_footer
}

launch_ros2_file() {
    local launch_target="$1"
    shift
    
    echo "════════════════════════════════════════════════════════════"
    echo "Starting Earth Rover - ROS2 Launch Mode: $launch_target"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    
    case "$launch_target" in
        vehicle_interface)
            echo "→ Launching MAVROS + Vehicle Interface..."
            exec ros2 launch "$EARTH_ROVER_HOME/launch/earth_rover.launch.py" mode:=minimal "$@"
            ;;
        full_system)
            echo "→ Launching Full System (MAVROS + Vehicle Interface + DeepGIS + Web)..."
            exec ros2 launch "$EARTH_ROVER_HOME/launch/earth_rover.launch.py" mode:=full "$@"
            ;;
        telemetry)
            echo "→ Launching Vehicle + Telemetry..."
            exec ros2 launch "$EARTH_ROVER_HOME/launch/earth_rover.launch.py" mode:=telemetry "$@"
            ;;
        custom)
            echo "→ Launching with custom configuration..."
            exec ros2 launch "$EARTH_ROVER_HOME/launch/earth_rover.launch.py" "$@"
            ;;
        *)
            echo "Error: Unknown ROS2 launch target: $launch_target"
            exit 1
            ;;
    esac
}

# ============================================================================
# Footer Messages
# ============================================================================

print_minimal_footer() {
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "✓ Minimal Earth Rover stack started"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    echo "Services running:"
    echo "  • MAVROS + Vehicle Interface"
    echo "  • ROSBridge WebSocket"
    echo "  • Vehicle Control Station"
    echo ""
    echo "Access Vehicle Control Station at:"
    echo "  http://localhost:8000"
    echo "  http://$(hostname -I | awk '{print $1}'):8000"
    echo ""
    echo "View logs in: $LOG_DIR"
    echo "════════════════════════════════════════════════════════════"
}

print_trike_footer() {
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "✓ Full Earth Rover Trike stack started"
    echo "════════════════════════════════════════════════════════════"
    echo ""
    echo "Services running:"
    echo "  • Serial Data Publisher"
    echo "  • MAVROS + Vehicle Interface"
    echo "  • Grasshopper3 Stereo Cameras"
    echo "  • Spectrometer Data Publisher"
    echo "  • RealSense Camera"
    echo "  • ROSBridge WebSocket"
    echo "  • Web Video Server"
    echo "  • Velodyne VLP-16 LiDAR"
    echo "  • DeepGIS GPS Publisher"
    echo "  • Vehicle Control Station"
    echo ""
    echo "Access Vehicle Control Station at:"
    echo "  http://localhost:8000"
    echo "  http://$(hostname -I | awk '{print $1}'):8000"
    echo ""
    echo "View logs in: $LOG_DIR"
    echo ""
    echo "To stop all services, run:"
    echo "  $EARTH_ROVER_HOME/scripts/startup/stop_trike_stack.sh"
    echo "════════════════════════════════════════════════════════════"
}

print_usage() {
    cat << EOF
Usage: $0 [MODE] [options]

SHELL MODES (multi-process with nohup):
  minimal           - MAVROS + ROSBridge + VCS Web Server (basic operation)
  trike             - Full trike stack with all sensors and services (default)

ROS2 LAUNCH MODES (single-process, unified launch file):
  vehicle_interface - MAVROS + Vehicle Interface only
  telemetry         - MAVROS + Vehicle Interface + DeepGIS Telemetry
  full_system       - All ROS2 services (MAVROS + Telemetry + Web)
  custom            - Custom configuration using enable flags

EXAMPLES:
  # Shell modes (background processes)
  $0 minimal
  $0 trike
  ROS_STARTUP_DELAY=15 $0 trike

  # ROS2 launch modes (foreground, single process)
  $0 vehicle_interface
  $0 telemetry deepgis_api_url:=http://192.168.0.186:8080
  $0 full_system enable_sdr:=true
  $0 custom enable_vehicle:=true enable_web:=true fcu_url:=/dev/ttyUSB0:57600

ENVIRONMENT VARIABLES:
  ROS_DISTRO        - ROS 2 distribution (default: humble)
  ROS_STARTUP_DELAY - Seconds to wait before starting (default: 0, recommended: 15)
  EARTH_ROVER_HOME  - Path to earth-rover directory (default: /home/jdas/earth-rover)
  ROS_BRIDGE_URL    - ROSBridge URL for VCS (default: ws://192.168.1.7:9090)
  VIDEO_SERVER_URL  - Video server URL for VCS (default: http://192.168.1.7:8080)

SHELL MODE DETAILS:

  minimal:
    • MAVROS + Vehicle Interface (Pixhawk connection)
    • ROSBridge WebSocket (for web interface)
    • Vehicle Control Station web server

  trike:
    • All services from minimal mode
    • Serial data publisher
    • Grasshopper3 stereo cameras
    • Spectrometer data publisher
    • RealSense camera
    • Web video server
    • Velodyne VLP-16 LiDAR
    • DeepGIS GPS publisher

ROS2 LAUNCH MODE PARAMETERS:
  fcu_url           - FCU connection URL
  enable_vehicle    - Enable MAVROS (true/false)
  enable_telemetry  - Enable DeepGIS telemetry (true/false)
  enable_web        - Enable web services (true/false)
  enable_sdr        - Enable Hydra SDR (true/false)
  deepgis_api_url   - DeepGIS API URL
  asset_name        - Asset/vehicle name
  rosbridge_port    - ROSBridge port (default: 9090)
  web_video_port    - Web video server port (default: 8080)

EOF
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    # Print header
    print_header
    
    # Setup ROS environment
    setup_ros_environment
    
    # Apply startup delay if configured
    apply_startup_delay
    
    # Change to earth-rover directory
    cd "$EARTH_ROVER_HOME"
    
    # Launch based on target mode
    case "$TARGET" in
        minimal)
            launch_minimal
            ;;
        trike)
            launch_trike
            ;;
        vehicle_interface|full_system|telemetry|custom)
            launch_ros2_file "$TARGET" "${@:2}"
            ;;
        help|--help|-h)
            print_usage
            exit 0
            ;;
        *)
            echo "Error: Unknown target mode: $TARGET"
            echo ""
            print_usage
            exit 1
            ;;
    esac
}

# Run main
main "$@"

