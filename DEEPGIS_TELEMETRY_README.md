# DeepGIS Telemetry Publisher

A ROS2 node that publishes MAVROS telemetry data to the DeepGIS cloud platform for real-time monitoring and post-flight analysis.

## Features

- **Automatic Session Management**: Creates and manages telemetry sessions
- **Multiple Data Sources**: 
  - Local position odometry (pose + velocity)
  - Raw GPS fix data
  - Estimated/filtered GPS position
- **Batch Mode**: Efficient batch uploads to reduce API calls
- **Real-time Mode**: Stream telemetry data with minimal latency
- **Configurable**: Flexible parameters for different use cases

## Installation

The node is automatically built with the `deepgis_vehicles` package:

```bash
cd ~/ros2_ws
colcon build --packages-select deepgis_vehicles --symlink-install
source install/setup.bash
```

## Usage

### Basic Usage

Launch with default settings:

```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py
```

### With Custom Vehicle ID

```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py vehicle_id:=my_drone_01
```

### With API Key Authentication

```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    vehicle_id:=my_drone_01 \
    api_key:=your_api_key_here
```

### Full System with MAVROS

```bash
# Terminal 1: Launch MAVROS and Vehicle Interface
ros2 launch deepgis_vehicles vehicle_interface.launch.py

# Terminal 2: Launch DeepGIS Telemetry Publisher
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py vehicle_id:=pixhawk_001
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `deepgis_api_url` | string | `https://deepgis.org` | Base URL for DeepGIS API |
| `api_key` | string | `""` | API key for authentication (optional) |
| `vehicle_id` | string | `pixhawk_001` | Unique identifier for the vehicle |
| `session_name` | string | `""` | Session name (auto-generated if empty) |
| `mavros_namespace` | string | `/mavros` | MAVROS namespace to subscribe to |
| `publish_rate` | float | `1.0` | Rate (Hz) to check and publish data |
| `upload_kbps` | float | `50.0` | Cap on compressed upload rate (kbit/s) |
| `upload_interval` | float | `5.0` | Seconds between batches once the spool is drained |
| `max_batch_rows` | int | `600` | Most samples per batch |
| `spool_path` | string | `~/.local/share/earth-rover/telemetry_spool.sqlite` | Local SQLite spool |
| `spool_max_rows` | int | `2000000` | Oldest samples are dropped beyond this |
| `stationary_interval` | float | `10.0` | Logging period (s) while parked |
| `stationary_speed` | float | `0.2` | Below this speed (m/s) the vehicle may be parked |
| `stationary_radius_m` | float | `2.0` | ...and within this distance (m) of the last logged fix |
| `session_id` | string | `""` | Fixed session id; empty = one session per boot, `<asset>_<start time>` |

## API Endpoints Used

The node interacts with the following DeepGIS API endpoints:

- `POST /api/telemetry/session/create/` - Create a new telemetry session
- `POST /api/telemetry/local-position-odom/` - Upload local position data
- `POST /api/telemetry/gps-fix-raw/` - Upload raw GPS fixes
- `POST /api/telemetry/gps-fix-estimated/` - Upload estimated GPS position
- `POST /api/telemetry/batch/` - Upload batch of mixed telemetry data

## Data Format

### Session Creation
```json
{
  "vehicle_id": "pixhawk_001",
  "session_name": "flight_20251124_212900",
  "start_time": "2025-11-24T21:29:00.123456",
  "metadata": {
    "source": "ros2_mavros",
    "node_name": "deepgis_telemetry_publisher"
  }
}
```

### Local Position Odometry
```json
{
  "session_id": "uuid-here",
  "timestamp": "2025-11-24T21:29:01.123456",
  "header": {...},
  "pose": {
    "position": {"x": 1.5, "y": 2.3, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "twist": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
  }
}
```

### GPS Fix Data
```json
{
  "session_id": "uuid-here",
  "timestamp": "2025-11-24T21:29:01.123456",
  "header": {...},
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 125.5,
  "position_covariance": [...],
  "position_covariance_type": 2,
  "status": {"status": 0, "service": 1}
}
```

## Subscribed Topics

The node subscribes to these MAVROS topics:

- `{mavros_namespace}/local_position/odom` (nav_msgs/Odometry)
- `{mavros_namespace}/global_position/raw/fix` (sensor_msgs/NavSatFix)
- `{mavros_namespace}/global_position/global` (sensor_msgs/NavSatFix)

## Logging Over Weak Links

Every sample goes into a local SQLite spool before anything is sent. An uploader thread sends
the oldest samples as one gzip-compressed batch (`Content-Encoding: gzip`) to
`/api/telemetry/batch/`, sleeps long enough to stay under `upload_kbps`, and deletes the samples
only after the server answers 201 or 207. While the link is down (no signal, a Starlink dish in
standby, the server restarting) samples wait in the spool and the uploader retries with backoff
from 5 s to 2 min, so an outage delays data rather than losing it.

The server keeps one record per `(session, timestamp_usec)`, so a batch resent after an unclear
failure is stored once. The session id is saved with the kernel boot id, so restarting the node
continues the same session; a reboot starts a new one. Timestamps come from the MAVROS message
header, in UTC.

Bandwidth: a parked rover sends ~470 B every 10 s (~0.4 kbit/s); at 1 Hz while moving, ~2 kbit/s.
A backlog drains at up to `upload_kbps`.

Inspect the spool:
```bash
python3 -c "import sqlite3,os; db=sqlite3.connect(os.path.expanduser('~/.local/share/earth-rover/telemetry_spool.sqlite')); print(db.execute('select kind,count(*) from samples group by kind').fetchall())"
journalctl --user -u er-deepgis-telemetry -f | grep -E 'Uploaded|failed'
```

## Troubleshooting

### Node fails to create session
- Check that `deepgis_api_url` is correct and reachable
- Verify API key if authentication is required
- Check network connectivity

### No data being published
- Verify MAVROS is running: `ros2 node list | grep mavros`
- Check MAVROS topics: `ros2 topic list | grep mavros`
- Verify data is flowing: `ros2 topic echo /mavros/local_position/odom`

### High network usage
- Lower `upload_kbps` (backlogs drain more slowly; live samples still go out)
- Raise `stationary_interval`, or lower `publish_rate`

## Development

### Testing Locally

You can test the node without connecting to the real API by setting up a local mock server or using the DeepGIS development API:

```bash
ros2 launch deepgis_vehicles deepgis_telemetry.launch.py \
    deepgis_api_url:=http://localhost:8000
```

### Custom API Integration

If you need to modify the data format or add additional telemetry sources, edit:
- `scripts/deepgis_telemetry_publisher.py` - Main node implementation
- `config/deepgis_telemetry.yaml` - Default configuration

## License

Same as the parent deepgis_vehicles package.

