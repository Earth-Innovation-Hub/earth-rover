#!/bin/bash
# ROS Startup Script for Earth Rover
# Consolidated startup script that sources ROS 2 and launches the rover stack
#
# Usage: ros_startup.sh [trike|full_system|vehicle_interface|minimal]
#
# Env:
#   ROS_DISTRO        — ROS 2 distro (default: humble)
#   ROS_STARTUP_DELAY — Seconds to wait before starting (e.g. for USB), default: 0
#   EARTH_ROVER_HOME  — Path to earth-rover directory (default: /home/jdas/earth-rover)
#
# Examples:
#   ./ros_startup.sh trike
#   ROS_STARTUP_DELAY=15 ./ros_startup.sh trike
#   ./ros_startup.sh full_system

set -e

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export EARTH_ROVER_HOME="${EARTH_ROVER_HOME:-/home/jdas/earth-rover}"
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export LOG_DIR="${EARTH_ROVER_HOME}/scripts/logs"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

echo "================================================"
echo "Earth Rover ROS Startup"
echo "================================================"
echo "Rover Home: $EARTH_ROVER_HOME"
echo "ROS Distro: $ROS_DISTRO"
echo "Log Dir:    $LOG_DIR"
echo "Target:     ${1:-trike}"
echo "================================================"

# Source ROS 2 environment
echo "[ros_startup] Sourcing ROS 2 $ROS_DISTRO environment..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace if it exists
if [ -f /home/jdas/ros2_ws/install/setup.bash ]; then
    echo "[ros_startup] Sourcing ROS 2 workspace..."
    source /home/jdas/ros2_ws/install/setup.bash
fi

# Optional startup delay for USB devices
if [[ -n "$ROS_STARTUP_DELAY" && "$ROS_STARTUP_DELAY" =~ ^[0-9]+$ && "$ROS_STARTUP_DELAY" -gt 0 ]]; then
    echo "[ros_startup] Waiting ${ROS_STARTUP_DELAY}s for USB/devices..."
    sleep "$ROS_STARTUP_DELAY"
fi

# Change to earth-rover directory
cd "$EARTH_ROVER_HOME"

# Launch based on target
case "${1:-trike}" in
    trike)
        echo "[ros_startup] Launching full trike stack..."
        exec "${SCRIPT_DIR}/run_trike_stack.sh" "${@:2}"
        ;;
    full_system)
        echo "[ros_startup] Launching full system (MAVROS + Vehicle Interface + DeepGIS)..."
        exec ros2 launch "$EARTH_ROVER_HOME/launch/full_system.launch.py" "${@:2}"
        ;;
    vehicle_interface)
        echo "[ros_startup] Launching vehicle interface only..."
        exec ros2 launch "$EARTH_ROVER_HOME/launch/vehicle_interface.launch.py" "${@:2}"
        ;;
    minimal)
        echo "[ros_startup] Launching minimal stack (MAVROS + ROSBridge + Web Server)..."
        exec "${SCRIPT_DIR}/run_minimal_stack.sh" "${@:2}"
        ;;
    *)
        echo "Usage: $0 [trike|full_system|vehicle_interface|minimal]"
        echo ""
        echo "Targets:"
        echo "  trike            - Full trike stack with all sensors and services"
        echo "  full_system      - MAVROS + Vehicle Interface + DeepGIS telemetry"
        echo "  vehicle_interface - MAVROS + Vehicle Interface only"
        echo "  minimal          - MAVROS + ROSBridge + VCS Web Server only"
        exit 1
        ;;
esac

