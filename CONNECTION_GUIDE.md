# Connecting to Pixhawk via MAVROS2

## Quick Start

### Method 1: Using the helper script (easiest)

```bash
# Auto-detect and connect (uses default baud rate 57600)
ros2 run deepgis-vehicles connect_pixhawk.sh

# Or specify baud rate
ros2 run deepgis-vehicles connect_pixhawk.sh 921600
```

### Method 2: Manual launch with serial-by-id (recommended)

```bash
# Find your Pixhawk device
ls -la /dev/serial/by-id/

# Launch with the device path (replace with your actual device)
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:57600"
```

### Method 3: Using direct device path

```bash
# For /dev/ttyACM0
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyACM0:57600"

# For /dev/ttyUSB0
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyUSB0:57600"
```

## Finding Your Pixhawk Device

### List all serial devices by ID:
```bash
ls -la /dev/serial/by-id/
```

### Check device permissions:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

### If you get permission denied errors:
```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, or run:
newgrp dialout
```

## Common Baud Rates

- **57600** - Default telemetry rate (most common)
- **921600** - High-speed telemetry (faster data)
- **115200** - Alternative standard rate

## Verify Connection

Once launched, check the connection status:

```bash
# In another terminal, check MAVROS state
ros2 topic echo /mavros/state

# Check vehicle connection status
ros2 topic echo /vehicle/connected

# View vehicle odometry
ros2 topic echo /vehicle/odometry
```

## Troubleshooting

### Device not found
- Ensure Pixhawk is powered on and connected via USB
- Check USB cable (some cables are power-only)
- Try different USB port

### Permission denied
- Add user to dialout group (see above)
- Check device permissions: `ls -l /dev/ttyACM*`

### Connection timeout
- Verify baud rate matches PX4 configuration
- Check PX4 parameter `SER_TEL1_BAUD` (usually 57600)
- Try different baud rates: 57600, 115200, 921600

### MAVROS not connecting
- Ensure PX4 is running and not in bootloader mode
- Check that MAVLink is enabled on the telemetry port
- Verify system ID matches (default is 1)

## Your Current Device

Based on your system, your Pixhawk is at:
```
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00
```

Quick connect command:
```bash
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:57600"
```

