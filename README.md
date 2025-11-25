# deepgis-vehicles

ROS2 package for connecting with Pixhawk PX4 autopilot using MAVROS2.

## Overview

This package provides a ROS2 interface to communicate with Pixhawk PX4 flight controllers via MAVROS2. It includes:

- **vehicle_interface_node**: A C++ node that interfaces with MAVROS2 to:
  - Subscribe to vehicle state, position, and sensor data
  - Publish commands (position setpoints, velocity commands)
  - Provide services for arming/disarming and mode changes
  - Publish processed vehicle data as standard ROS2 topics

## Dependencies

- ROS2 (Humble/Iron/Rolling)
- MAVROS2 (`mavros`, `mavros_msgs`, `mavros_extras`)
- Standard ROS2 packages: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`
- TF2 for coordinate transformations

## Installation

### Install MAVROS2

Before building this package, you need to install MAVROS2:

**For ROS2 Humble (Ubuntu 22.04):**
```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
```

**For ROS2 Iron (Ubuntu 22.04):**
```bash
sudo apt update
sudo apt install ros-iron-mavros ros-iron-mavros-extras ros-iron-mavros-msgs
```

**For ROS2 Rolling (Ubuntu 22.04/24.04):**
```bash
sudo apt update
sudo apt install ros-rolling-mavros ros-rolling-mavros-extras ros-rolling-mavros-msgs
```

**Note:** If MAVROS2 packages are not available via apt, you may need to build them from source. See the [MAVROS2 GitHub repository](https://github.com/mavlink/mavros) for instructions.

### Install GeographicLib datasets (required for MAVROS2)

MAVROS2 requires GeographicLib datasets for coordinate transformations:

```bash
sudo geographiclib-get-geoids egm96-5
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select deepgis-vehicles
source install/setup.bash
```

## Usage

### Launch with PX4 SITL (Software In The Loop)

For testing with PX4 SITL:

```bash
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="udp://:14540@127.0.0.1:14557"
```

### Launch with Physical Pixhawk

**Recommended: Using serial-by-id (stable device identification)**

First, find your Pixhawk device:
```bash
ls -la /dev/serial/by-id/
```

Then launch with the serial-by-id path (recommended for stability):
```bash
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:57600"
```

**Alternative: Using direct device paths**

For a Pixhawk connected via USB:
```bash
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyUSB0:57600"
```

For a Pixhawk connected via ACM:
```bash
ros2 launch deepgis-vehicles vehicle_interface.launch.py \
    fcu_url:="/dev/ttyACM0:57600"
```

**Note:** The serial-by-id method (`/dev/serial/by-id/...`) is recommended because it provides a stable path that doesn't change when the device is plugged into different USB ports.

### Launch Parameters

- `fcu_url`: Connection URL to the flight controller
  - SITL: `udp://:14540@127.0.0.1:14557`
  - Serial-by-id (recommended): `/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:57600`
  - USB: `/dev/ttyUSB0:57600`
  - ACM: `/dev/ttyACM0:57600`
  - TCP: `tcp://127.0.0.1:5760`
  
  **Baud rates:** Common baud rates for Pixhawk are 57600 (default), 921600 (high-speed), or 115200
- `gcs_url`: Ground Control Station URL (default: `udp://@127.0.0.1:14550`)
- `tgt_system`: Target system ID (default: `1`)
- `tgt_component`: Target component ID (default: `1`)
- `mavros_namespace`: MAVROS namespace (default: `/mavros`)

## Topics

### Subscribed (from MAVROS2)

- `/mavros/state` - Vehicle state (armed, mode, connected)
- `/mavros/local_position/pose` - Local position estimate
- `/mavros/global_position/global` - Global GPS position

### Published (to MAVROS2)

- `/mavros/setpoint_position/local` - Position setpoint commands
- `/mavros/setpoint_velocity/cmd_vel` - Velocity commands
- `/mavros/setpoint_raw/local` - Raw position target

### Published (processed data)

- `vehicle/odometry` - Vehicle odometry (nav_msgs/Odometry)
- `vehicle/connected` - Connection status (std_msgs/Bool)

## Services

The node provides access to MAVROS2 services:

- `/mavros/cmd/arming` - Arm/disarm the vehicle
- `/mavros/set_mode` - Change flight mode

## Example: Arming and Taking Off

```bash
# In one terminal, launch the interface
ros2 launch deepgis-vehicles vehicle_interface.launch.py

# In another terminal, arm the vehicle
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Set to OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
    "{custom_mode: 'OFFBOARD'}"

# Publish position setpoint (example: 5m altitude)
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, \
     pose: {position: {x: 0.0, y: 0.0, z: 5.0}, \
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## Monitoring

Check vehicle connection status:

```bash
ros2 topic echo /mavros/state
ros2 topic echo /vehicle/connected
ros2 topic echo /vehicle/odometry
```

## Configuration

Edit `config/mavros_config.yaml` to customize MAVROS2 parameters.

## Notes

- Ensure MAVROS2 is installed: `sudo apt install ros-<distro>-mavros ros-<distro>-mavros-extras`
- For USB connections, you may need to add your user to the `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- The node automatically handles connection status and logs important state changes
- Position setpoints should be published at a minimum rate (typically 10-30 Hz) for OFFBOARD mode

## License

MIT

