#!/bin/bash
# RTL-SDR ROS2 Package Build and Test Script

set -e  # Exit on error

echo "=============================================="
echo "RTL-SDR ROS2 Package Build and Test"
echo "=============================================="
echo ""

# Navigate to workspace
cd ~/ros2_ws

echo "1. Cleaning previous build..."
rm -rf build/rtlsdr_ros2 install/rtlsdr_ros2
echo "   ✓ Clean complete"
echo ""

echo "2. Building package..."
colcon build --packages-select rtlsdr_ros2 --symlink-install
echo "   ✓ Build complete"
echo ""

echo "3. Sourcing install..."
source install/setup.bash
echo "   ✓ Source complete"
echo ""

echo "4. Checking executables..."
if [ -f "install/rtlsdr_ros2/lib/rtlsdr_ros2/rtlsdr_reader" ]; then
    echo "   ✓ rtlsdr_reader found"
else
    echo "   ✗ rtlsdr_reader NOT found"
    exit 1
fi

if [ -f "install/rtlsdr_ros2/lib/rtlsdr_ros2/spectrum_visualizer" ]; then
    echo "   ✓ spectrum_visualizer found"
else
    echo "   ✗ spectrum_visualizer NOT found"
    exit 1
fi

if [ -f "install/rtlsdr_ros2/lib/rtlsdr_ros2/signal_recorder" ]; then
    echo "   ✓ signal_recorder found"
else
    echo "   ✗ signal_recorder NOT found"
    exit 1
fi
echo ""

echo "5. Checking messages..."
if ros2 interface show rtlsdr_ros2/msg/RtlsdrSignal > /dev/null 2>&1; then
    echo "   ✓ RtlsdrSignal message found"
else
    echo "   ✗ RtlsdrSignal message NOT found"
    exit 1
fi

if ros2 interface show rtlsdr_ros2/msg/RtlsdrSpectrum > /dev/null 2>&1; then
    echo "   ✓ RtlsdrSpectrum message found"
else
    echo "   ✗ RtlsdrSpectrum message NOT found"
    exit 1
fi
echo ""

echo "6. Checking launch file..."
if [ -f "install/rtlsdr_ros2/share/rtlsdr_ros2/launch/rtlsdr_launch.py" ]; then
    echo "   ✓ Launch file found"
else
    echo "   ✗ Launch file NOT found"
    exit 1
fi
echo ""

echo "7. Checking config file..."
if [ -f "install/rtlsdr_ros2/share/rtlsdr_ros2/config/rtlsdr_config.yaml" ]; then
    echo "   ✓ Config file found"
else
    echo "   ✗ Config file NOT found"
    exit 1
fi
echo ""

echo "=============================================="
echo "✓ All checks passed!"
echo "=============================================="
echo ""
echo "Package is ready to use. Try:"
echo "  ros2 launch rtlsdr_ros2 rtlsdr_launch.py"
echo ""
echo "Note: You'll need an RTL-SDR device connected to run the node."
echo ""

