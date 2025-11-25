# Quick Start: DeepGIS Telemetry

Get your Pixhawk telemetry streaming to DeepGIS in minutes!

## Prerequisites

1. MAVROS2 installed (see `INSTALL_MAVROS2.md`)
2. Pixhawk connected via USB-TTL adapter
3. DeepGIS account (optional: get API key from https://deepgis.org)

## Option 1: Full System (Recommended)

Launch everything together - MAVROS, Vehicle Interface, and DeepGIS Telemetry:

```bash
cd ~/ros2_ws
source install/setup.bash

# Basic launch (anonymous telemetry)
ros2 launch deepgis_vehicles full_system.launch.py

# With vehicle ID
ros2 launch deepgis_vehicles full_system.launch.py vehicle_id:=my_drone_01

# With API key for authenticated access
ros2 launch deepgis_vehicles full_system.launch.py \
    vehicle_id:=my_drone_01 \
    api_key:=your_api_key_here
```

## Option 2: Separate Launches

If you prefer to launch components separately:

### Terminal 1: Vehicle Interface (MAVROS)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch deepgis_vehicles vehicle_interface.launch.py
```

### Terminal 2: DeepGIS Telemetry Publisher
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py vehicle_id:=my_drone_01
```

## Verify It's Working

### Check Active Nodes
```bash
ros2 node list
```

You should see:
- `/mavros`
- `/vehicle_interface_node`
- `/deepgis_telemetry_publisher`

### Monitor Telemetry Publishing
```bash
# Watch the telemetry publisher logs
ros2 node list
ros2 topic list | grep mavros

# Check if data is being received
ros2 topic echo /mavros/local_position/odom --once
ros2 topic echo /mavros/global_position/global --once
```

### View Logs
```bash
# Watch the telemetry node output
ros2 run deepgis_vehicles deepgis_telemetry_publisher.py --ros-args --log-level debug
```

## Configuration Examples

### High-Frequency Data Collection (10 Hz)
```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    vehicle_id:=research_drone \
    publish_rate:=10.0 \
    batch_size:=50
```

### Real-Time Streaming (Low Latency)
```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    vehicle_id:=demo_drone \
    enable_batch_mode:=false \
    publish_rate:=5.0
```

### Testing with Local API
```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    deepgis_api_url:=http://localhost:8000 \
    vehicle_id:=test_vehicle
```

## View Your Data

Once telemetry is publishing:

1. Open your browser to https://deepgis.org
2. Navigate to "Telemetry" or "Live Flights"
3. Find your vehicle by ID (e.g., `pixhawk_001`)
4. View real-time position, GPS, and velocity data

## Troubleshooting

### Problem: "Failed to create session"

**Solution**: Check your internet connection and API URL
```bash
# Test connectivity
ping deepgis.org

# Verify API endpoint
curl https://deepgis.org/api/telemetry/
```

### Problem: "No data being published"

**Solution**: Verify MAVROS is receiving data from Pixhawk
```bash
# Check MAVROS connection
ros2 topic echo /mavros/state --once

# Check if data is flowing
ros2 topic hz /mavros/local_position/odom
ros2 topic hz /mavros/global_position/global
```

### Problem: Node crashes or fails to start

**Solution**: Check logs for detailed error messages
```bash
# View recent logs
ros2 run deepgis_vehicles deepgis_telemetry_publisher.py --ros-args --log-level debug

# Or check system logs
journalctl -f | grep deepgis
```

### Problem: High network bandwidth usage

**Solution**: Adjust publish rate and enable batch mode
```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    publish_rate:=0.5 \
    batch_size:=20 \
    enable_batch_mode:=true
```

## What Data is Sent?

The telemetry publisher sends:

1. **Local Position Odometry** (`/mavros/local_position/odom`)
   - 3D position (x, y, z) in local frame
   - Orientation (quaternion)
   - Linear velocity (vx, vy, vz)
   - Angular velocity (roll rate, pitch rate, yaw rate)

2. **Raw GPS Fix** (`/mavros/global_position/raw/fix`)
   - Latitude, Longitude, Altitude
   - GPS fix status
   - Position accuracy/covariance

3. **Estimated GPS Position** (`/mavros/global_position/global`)
   - Filtered/estimated global position
   - Same format as raw GPS

All data includes:
- Timestamps (ROS time and wall clock time)
- Session ID for grouping flights
- Vehicle ID for identification

## Advanced: Custom Configuration File

Create `my_config.yaml`:

```yaml
deepgis_telemetry_publisher:
  ros__parameters:
    deepgis_api_url: "https://deepgis.org"
    api_key: "your_api_key"
    vehicle_id: "custom_drone"
    mavros_namespace: "/mavros"
    publish_rate: 2.0
    enable_batch_mode: true
    batch_size: 15
```

Launch with custom config:
```bash
ros2 run deepgis_vehicles deepgis_telemetry_publisher.py \
    --ros-args --params-file my_config.yaml
```

## Next Steps

- See `DEEPGIS_TELEMETRY_README.md` for detailed documentation
- Check `CONNECTION_GUIDE.md` for Pixhawk connection details
- Visit https://deepgis.org/docs for API documentation

## Support

For issues or questions:
1. Check the logs: `ros2 node list` and view output
2. Verify MAVROS connection: `ros2 topic list | grep mavros`
3. Test with local mock API server first
4. Review DeepGIS API documentation

