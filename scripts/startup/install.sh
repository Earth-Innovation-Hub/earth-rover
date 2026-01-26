#!/bin/bash
# Earth Rover - Startup Scripts Installation
# This script installs the Earth Rover startup scripts and configures autostart

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EARTH_ROVER_HOME="$(dirname $(dirname "$SCRIPT_DIR"))"

echo "================================================"
echo "Earth Rover Startup Scripts Installation"
echo "================================================"
echo "Earth Rover Home: $EARTH_ROVER_HOME"
echo ""

# Make all scripts executable
echo "[1/4] Making scripts executable..."
chmod +x "$SCRIPT_DIR"/*.sh
echo "  ✓ Scripts are now executable"

# Install systemd user service
echo "[2/4] Installing systemd user service..."
mkdir -p ~/.config/systemd/user
cp "$SCRIPT_DIR/ros-trike.service" ~/.config/systemd/user/
systemctl --user daemon-reload
echo "  ✓ Service installed: ros-trike.service"
echo ""
echo "  To enable auto-start on login:"
echo "    systemctl --user enable ros-trike.service"
echo "  To start now:"
echo "    systemctl --user start ros-trike.service"
echo "  To check status:"
echo "    systemctl --user status ros-trike.service"

# Install desktop autostart file
echo "[3/4] Installing desktop autostart file..."
mkdir -p ~/.config/autostart
cp "$SCRIPT_DIR/ros-trike.desktop" ~/.config/autostart/
echo "  ✓ Desktop file installed: ~/.config/autostart/ros-trike.desktop"
echo "  (Will auto-start on graphical login)"

# Create log directory
echo "[4/4] Creating log directory..."
mkdir -p "$EARTH_ROVER_HOME/scripts/logs"
echo "  ✓ Log directory: $EARTH_ROVER_HOME/scripts/logs"

echo ""
echo "================================================"
echo "✓ Installation Complete!"
echo "================================================"
echo ""
echo "Available startup options:"
echo "  1. Auto-start on login (desktop):"
echo "     - Already configured via ~/.config/autostart/ros-trike.desktop"
echo ""
echo "  2. Auto-start via systemd (optional):"
echo "     systemctl --user enable ros-trike.service"
echo ""
echo "  3. Manual start:"
echo "     $EARTH_ROVER_HOME/scripts/startup/ros_startup.sh [target]"
echo ""
echo "Available targets:"
echo "  - trike            : Full stack with all sensors"
echo "  - minimal          : MAVROS + ROSBridge + VCS only"
echo "  - full_system      : MAVROS + DeepGIS telemetry"
echo "  - vehicle_interface: MAVROS only"
echo ""
echo "Useful commands:"
echo "  Start:  $EARTH_ROVER_HOME/scripts/startup/ros_startup.sh trike"
echo "  Stop:   $EARTH_ROVER_HOME/scripts/startup/stop_trike_stack.sh"
echo "  Logs:   ls -lh $EARTH_ROVER_HOME/scripts/logs/"
echo "================================================"

