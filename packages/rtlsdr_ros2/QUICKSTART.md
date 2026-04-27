# RTL-SDR ROS2 Quick Start Guide

## Installation (5 minutes)

### 1. Install Dependencies
```bash
cd ~/ros2_ws/src/rtlsdr_ros2
./install_dependencies.sh
```

Or manually:
```bash
# System packages
sudo apt-get install rtl-sdr librtlsdr-dev

# Python packages
pip3 install pyrtlsdr numpy
```

### 2. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select rtlsdr_ros2
source install/setup.bash
```

### 3. Test RTL-SDR Device
```bash
# Unplug and replug your RTL-SDR first
rtl_test
# Press Ctrl+C to stop after a few seconds
```

## Quick Usage

### Option 1: Default Settings (FM Radio Band)
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py
```

### Option 2: Custom Frequency
```bash
# Weather satellite at 137.5 MHz
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=137.5e6

# ADS-B aircraft at 1090 MHz
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=1090.0e6

# ISM 433 MHz
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=433.92e6
```

### Option 3: View Spectrum Data
In terminal 1:
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py
```

In terminal 2:
```bash
ros2 run rtlsdr_ros2 spectrum_visualizer
```

## Verify It's Working

### Check Topics
```bash
ros2 topic list
# Should show:
#   /rtlsdr/signal
#   /rtlsdr/spectrum
```

### Monitor Data
```bash
# Check publishing rate
ros2 topic hz /rtlsdr/spectrum

# See spectrum peaks
ros2 topic echo /rtlsdr/spectrum --field peak_frequency
```

## Troubleshooting

**"Device not found"**
- Unplug and replug RTL-SDR
- Check: `lsusb | grep RTL`
- Test: `rtl_test`

**"Permission denied"**
- Rerun: `./install_dependencies.sh`
- Unplug and replug device

**No data**
- Close other RTL-SDR applications
- Check device with: `rtl_test`

## Next Steps

- Read full [README.md](README.md) for detailed documentation
- Edit [config/rtlsdr_config.yaml](config/rtlsdr_config.yaml) for custom settings
- Explore message definitions in `msg/` directory

## Example Applications

**FM Radio Reception**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=98.0e6 sample_rate:=2.4e6
```

**Air Traffic Control**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=127.5e6
```

**IoT Device Monitoring (433 MHz)**
```bash
ros2 launch rtlsdr_ros2 rtlsdr_launch.py center_frequency:=433.92e6 gain:=40.0
```

For more examples, see the [README.md](README.md).

