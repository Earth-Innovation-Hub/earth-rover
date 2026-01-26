#!/bin/bash
# Helper script to connect to Pixhawk via MAVROS2
# This script finds the Pixhawk device and launches the vehicle interface

# Find Pixhawk devices
echo "Searching for Pixhawk devices..."
PIXHAWK_DEVICES=$(ls /dev/serial/by-id/*PX4* 2>/dev/null | head -1)

if [ -z "$PIXHAWK_DEVICES" ]; then
    echo "No Pixhawk device found in /dev/serial/by-id/"
    echo "Checking /dev/ttyACM* and /dev/ttyUSB*..."
    PIXHAWK_DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1)
    if [ -z "$PIXHAWK_DEVICES" ]; then
        echo "ERROR: No Pixhawk device found!"
        echo "Please ensure your Pixhawk is connected via USB."
        exit 1
    fi
    echo "Found device: $PIXHAWK_DEVICES"
    DEVICE_PATH="$PIXHAWK_DEVICES"
else
    echo "Found Pixhawk device: $PIXHAWK_DEVICES"
    DEVICE_PATH="$PIXHAWK_DEVICES"
fi

# Default baud rate (can be overridden)
BAUD_RATE=${1:-57600}

# Construct FCU URL
FCU_URL="${DEVICE_PATH}:${BAUD_RATE}"

echo "Connecting to Pixhawk at: $FCU_URL"
echo "Launching MAVROS2 and vehicle interface..."
echo ""

# Launch the vehicle interface
ros2 launch deepgis-vehicles vehicle_interface.launch.py fcu_url:="$FCU_URL"

