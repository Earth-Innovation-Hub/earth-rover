# Earth Rover Startup Scripts

Consolidated ROS 2 startup scripts for the Earth Rover platform.

## Directory Structure

```
scripts/startup/
├── install.sh              # Installation script (run this first)
├── ros_startup.sh          # Main startup router
├── run_trike_stack.sh      # Full trike stack launcher
├── run_minimal_stack.sh    # Minimal stack launcher
├── stop_trike_stack.sh     # Graceful shutdown script
├── ros-trike.service       # Systemd user service file
├── ros-trike.desktop       # Desktop autostart file
└── README.md               # This file

scripts/logs/               # Log files directory (created automatically)
```

## Quick Start

### 1. Install Startup Scripts

```bash
cd /home/jdas/earth-rover/scripts/startup
./install.sh
```

This will:
- Make all scripts executable
- Install systemd service to `~/.config/systemd/user/`
- Install desktop autostart file to `~/.config/autostart/`
- Create log directory

### 2. Choose Startup Method

#### Option A: Auto-start on Login (Desktop)
Already configured by `install.sh`. The stack will auto-start when you log in to the graphical session.

#### Option B: Auto-start via Systemd (Optional)
```bash
systemctl --user enable ros-trike.service
systemctl --user start ros-trike.service
```

#### Option C: Manual Start
```bash
./ros_startup.sh [target]
```

## Available Targets

### `trike` (Default - Full Stack)
Launches all services:
- MAVROS (Pixhawk connection)
- Serial data publisher
- Grasshopper3 stereo cameras
- RealSense camera
- Spectrometer data publisher
- Velodyne VLP-16 LiDAR
- ROSBridge WebSocket server
- Web video server
- DeepGIS GPS publisher
- Vehicle Control Station web server

**Usage:**
```bash
./ros_startup.sh trike
# or with delay
ROS_STARTUP_DELAY=15 ./ros_startup.sh trike
```

### `minimal` (Essential Services Only)
Launches minimal stack:
- MAVROS (Pixhawk)
- ROSBridge WebSocket
- Vehicle Control Station web server

**Usage:**
```bash
./ros_startup.sh minimal
```

### `full_system` (ROS Launch File)
Uses the ROS 2 launch file for:
- MAVROS + Vehicle Interface
- DeepGIS telemetry publisher

**Usage:**
```bash
./ros_startup.sh full_system
# or with custom parameters
./ros_startup.sh full_system fcu_url:=/dev/ttyUSB0:57600
```

### `vehicle_interface` (MAVROS Only)
Launches only MAVROS and vehicle interface node.

**Usage:**
```bash
./ros_startup.sh vehicle_interface
```

## Environment Variables

- `ROS_DISTRO` - ROS 2 distribution (default: humble)
- `ROS_STARTUP_DELAY` - Seconds to wait before starting (default: 0, recommended: 15)
- `EARTH_ROVER_HOME` - Path to earth-rover directory (default: /home/jdas/earth-rover)

## Managing Services

### Start Services
```bash
# Full stack
/home/jdas/earth-rover/scripts/startup/ros_startup.sh trike

# Minimal stack
/home/jdas/earth-rover/scripts/startup/ros_startup.sh minimal
```

### Stop Services
```bash
/home/jdas/earth-rover/scripts/startup/stop_trike_stack.sh
```

### View Logs
```bash
# List all logs
ls -lh /home/jdas/earth-rover/scripts/logs/

# Tail a specific log
tail -f /home/jdas/earth-rover/scripts/logs/mavros_px4.log

# View Django VCS log
tail -f /home/jdas/earth-rover/scripts/logs/django_vcs.log
```

### Check Systemd Service Status
```bash
systemctl --user status ros-trike.service
systemctl --user stop ros-trike.service
systemctl --user start ros-trike.service
systemctl --user restart ros-trike.service
```

## Log Files

All logs are written to: `/home/jdas/earth-rover/scripts/logs/`

Log files:
- `mavros_px4.log` - MAVROS flight controller connection
- `rosbridge.log` - ROSBridge WebSocket server
- `django_vcs.log` - Vehicle Control Station web server
- `web_video_server.log` - Camera streaming server
- `velodyne.log` - Velodyne LiDAR
- `realsense.log` - RealSense camera
- `grasshopper_stereo.log` - Grasshopper stereo cameras
- `spectrometer.log` - Spectrometer data publisher
- `ros_publish_serial.log` - Serial data publisher
- `deepgis_gps.log` - DeepGIS GPS publisher

## Troubleshooting

### Services won't start
1. Check USB devices are connected:
   ```bash
   ls /dev/serial/by-id/
   ```

2. Increase startup delay:
   ```bash
   ROS_STARTUP_DELAY=30 ./ros_startup.sh trike
   ```

3. Check ROS environment:
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/jdas/ros2_ws/install/setup.bash
   ros2 node list
   ```

### Check individual service logs
```bash
# Check which services are running
pgrep -af "mavros|rosbridge|manage.py"

# View specific log
tail -f /home/jdas/earth-rover/scripts/logs/mavros_px4.log
```

### Pixhawk not connecting
```bash
# Use helper script to auto-detect Pixhawk
/home/jdas/earth-rover/scripts/connect_pixhawk.sh
```

### Web interface not accessible
1. Check Django is running:
   ```bash
   pgrep -f "manage.py runserver"
   tail -f /home/jdas/earth-rover/scripts/logs/django_vcs.log
   ```

2. Check ROSBridge is running:
   ```bash
   pgrep -f rosbridge
   tail -f /home/jdas/earth-rover/scripts/logs/rosbridge.log
   ```

3. Access at: http://localhost:8000 or http://YOUR_IP:8000

## Migration from Old Scripts

If you have old startup scripts, you can disable them:

```bash
# Disable old systemd service
systemctl --user disable ros-trike.service
systemctl --user stop ros-trike.service

# Remove old autostart file
rm ~/.config/autostart/ros-trike.desktop

# Then run the new installation
cd /home/jdas/earth-rover/scripts/startup
./install.sh
```

## Customization

To customize the startup behavior, edit:
- `run_trike_stack.sh` - Modify services or add new ones
- `run_minimal_stack.sh` - Change minimal stack configuration
- `ros_startup.sh` - Add new targets or modify startup logic

## Support

For issues or questions:
- Check logs in `/home/jdas/earth-rover/scripts/logs/`
- Review ROS topics: `ros2 topic list`
- Check ROS nodes: `ros2 node list`
- Verify connections: `ros2 node info /mavros`

