#!/bin/bash
# RTL-SDR ROS2 Package Dependency Installation Script

echo "=== RTL-SDR ROS2 Package Dependency Installer ==="
echo ""

# Check if running with sudo for apt operations
if [ "$EUID" -ne 0 ]; then 
    echo "Note: This script will use sudo for system package installation"
    echo ""
fi

# Install RTL-SDR system packages
echo "1. Installing RTL-SDR system packages..."
sudo apt-get update
sudo apt-get install -y rtl-sdr librtlsdr-dev

# Install Python dependencies
echo ""
echo "2. Installing Python dependencies..."
pip3 install --user pyrtlsdr numpy

# Set up udev rules
echo ""
echo "3. Setting up udev rules for RTL-SDR access..."
sudo bash -c 'cat > /etc/udev/rules.d/20-rtlsdr.rules << EOF
# RTL-SDR
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", MODE:="0666"
EOF'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Next steps:"
echo "1. Unplug and replug your RTL-SDR device"
echo "2. Test with: rtl_test"
echo "3. Build the ROS2 package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select rtlsdr_ros2"
echo "   source install/setup.bash"
echo ""

