# Vehicle Control Station

A real-time web-based monitoring and control interface for the Earth Rover platform. This Django application provides comprehensive visualization of sensor data, camera feeds, and control capabilities for ROS-based robotic systems.

## Features

### 📹 Real-Time Camera Feeds
- **Stereo Camera System**: Live streaming from left and right stereo cameras
- **Fisheye Camera**: Wide-angle view for enhanced situational awareness
- Direct video stream integration via web-mjpeg-server

### 📊 Sensor Data Visualization

#### GPS Navigation
- Interactive map display with real-time GPS tracking
- Path visualization showing vehicle trajectory
- Powered by Leaflet.js and OpenStreetMap

#### LiDAR Sensors
- **2D Laser Scan Visualization**: Real-time polar plot of laser range data
  - Visual range indicators
  - Point count and maximum range display
  - Interactive canvas rendering
  
- **3D Velodyne Point Cloud**: Full 3D visualization using Three.js
  - Interactive controls (rotate, zoom, pan)
  - Auto-rotate mode
  - Real-time point cloud updates
  - Frame ID validation

#### Visual Odometry
- Real-time position tracking
- 2D trajectory plotting
- Pose estimation visualization

#### Spectrometer Data
- Multi-wavelength intensity plotting
- Real-time spectral analysis
- D3.js-powered visualizations

#### Laser Ranger
- Distance measurement display
- Historical data plotting
- Real-time value updates

### 🎥 ROS Topic Recording

#### Recording Controls
- **One-Click Recording**: Start/stop recording all ROS topics from the web interface
- **systemd-backed**: drives `er-record.service` (record_bag.launch.py)
- **Timestamped Output**: Automatically organized rosbags with timestamps
- **Status Indicators**: Visual feedback for recording state
- **Process Management**: Graceful start/stop with proper cleanup

#### Storage Management
- **Real-Time Disk Space Monitoring**: Visual progress bar showing storage capacity
- **Color-Coded Alerts**:
  - Green: < 70% used (plenty of space)
  - Orange: 70-90% used (moderate space)
  - Red: > 90% used (warning - low space)
- **Auto-Update**: Disk space refreshed every 10 seconds

### 🌐 WebSocket Integration
- Real-time ROS bridge connection (rosbridge_suite)
- Connection status indicators
- Automatic reconnection handling
- Low-latency data streaming

## Architecture

### Technology Stack
- **Backend**: Django 4.2 LTS (Python 3.12), WhiteNoise
- **Frontend**: 
  - HTML5/CSS3/JavaScript
  - D3.js for data visualization
  - Three.js for 3D rendering
  - Leaflet.js for mapping
  - ROSLib.js for ROS communication
- **Communication**: WebSocket (rosbridge)
- **Video Streaming**: web-mjpeg-server

### API Endpoints

| Endpoint | Method | What it does |
|---|---|---|
| `/` | GET | Dashboard (cameras, map, LiDAR, spectrometer, ADS-B, recording) |
| `/mission/` | GET | Live chart of the rover's systemd units from `scripts/startup/mission.yaml` |
| `/api/mission/` | GET | Manifest + `systemctl --user show` state for every unit (one call) |
| `/api/system-info/` | GET | Free space on the bag disk and CPU percent |
| `/api/recording/status/` | GET | Whether `er-record.service` is running, newest bag directory |
| `/api/recording/start/` | POST | `systemctl --user start er-record.service` (record_bag.launch.py, mcap) |
| `/api/recording/stop/` | POST | Stops it; SIGINT closes the bag cleanly |
| `/api/ros-nodes/` | GET | `ros2 node list` |

## Running on the rover

The console is part of the always-on stack: `er-console.target` runs
`er-rosbridge` (:9090), `er-web-video` (:8080) and `er-vcs` (:8000), generated
from `scripts/startup/mission.yaml`. They listen on `${VCS_BIND_ADDRESS}`
(set in `~/.config/earth-rover/secrets.env`); the rover's ufw rules admit those
ports from loopback and the cable network only, not from wifi.

One-time setup:

```bash
vehicle_control_station/setup_venv.sh      # Django + WhiteNoise in ~/.local/share/earth-rover/vcs-venv
# rosbridge_server and web_video_server come from apt (ros-jazzy-*), see scripts/startup/README.md
```

`SECRET_KEY` must be in `secrets.env`. Optional environment:

- `VCS_DEBUG=1`: Django debug pages and static autorefresh.
- `VCS_ALLOWED_HOSTS`: comma-separated Host names (default `*`).
- `ROS_BRIDGE_URL`, `VIDEO_SERVER_URL`: override the URLs the page uses. By default they are the host that served the page on :9090 / :8080.
- `RECORD_BAG_DIR`: where the disk gauge looks (default `~/earth-rover-bags`).

Day to day:

```bash
systemctl --user restart er-vcs           # after editing Python or templates
journalctl --user -u er-vcs -f
```

Static files (`vehicle_control_station/static/`) are served by WhiteNoise
straight from the source tree; there is no `collectstatic` step and no database.
Leaflet, d3 and three.js are vendored under `static/vehicle-control-station/vendor/`
so the page loads without internet (the OpenStreetMap tiles still need it).

For a quick local run off the rover:

```bash
SECRET_KEY=dev VCS_DEBUG=1 ~/.local/share/earth-rover/vcs-venv/bin/python manage.py runserver 127.0.0.1:8000
```

## Project Structure

```
vehicle_control_station/
├── manage.py
├── requirements.txt               # Django 4.2 LTS, WhiteNoise
├── setup_venv.sh                  # builds the venv the er-vcs unit uses
├── templates/
│   ├── home.html                  # dashboard
│   └── mission.html               # unit tier chart
└── vehicle_control_station/
    ├── settings.py                # env-driven, DEBUG off by default
    ├── urls.py
    ├── views.py                   # dashboard, system info, recording, ROS nodes
    ├── mission_views.py           # mission.yaml + systemd state
    └── static/vehicle-control-station/
        ├── vcs-app.js, vcs-styles.css, roslib.min.js
        └── vendor/                # leaflet, d3, three.js
```

## Troubleshooting

### WebSocket Connection Issues
- `systemctl --user status er-rosbridge`; `journalctl --user -u er-rosbridge`
- From wifi the console ports are refused by design (ufw); use the cable network or the rover's own display

### Camera Feeds Not Showing
- `systemctl --user status er-web-video er-grasshopper`
- `ros2 topic hz /stereo/left/image_raw`

### Recording Fails to Start
- `journalctl --user -u er-record`: the unit refuses to start with less than 20 GB free on the bag disk

### Point Cloud Not Rendering
- Verify Velodyne frame_id is set to "velodyne" in the driver launch file
- Check `/velodyne_points` topic is publishing: `ros2 topic echo /velodyne_points --once`
- Ensure point cloud contains valid data

## Security Considerations

- **CSRF Protection**: Enabled for all POST requests
- **Process Isolation**: Recording runs as its own systemd unit
- **Network**: no login; access is limited to loopback and the cable network by ufw. rosbridge exposes the whole ROS graph (including MAVROS commands) to anyone who can reach :9090
- **Network Security**: Configure firewall rules appropriately for production

⚠️ **Note**: This is intended for internal network use. Do not expose directly to the internet without proper authentication and encryption.

## Performance Optimization

- Point cloud downsampling for large datasets
- Canvas-based rendering for 2D laser scans
- Throttled disk space polling (10-second intervals)
- Efficient WebGL rendering for 3D visualizations

## Known Limitations

- Single concurrent recording only
- Fixed recording directory
- Records all topics (no selective recording UI)
- No built-in recording duration limits
- Camera streams depend on external MJPEG server

## Future Enhancements

- [ ] Selective topic recording interface
- [ ] Custom output directory selection
- [ ] Recording duration timer and auto-stop
- [ ] Historical rosbag playback interface
- [ ] Multi-user support with authentication
- [ ] Real-time bandwidth monitoring per topic
- [ ] Automatic cleanup of old rosbags
- [ ] Recording compression options
- [ ] Topic remapping configuration UI

## Contributing

When contributing to this project:
1. Test with both ROS1 and ROS2 if applicable
2. Ensure CSRF tokens are properly handled in new API endpoints
3. Update this README with new features
4. Follow Django best practices

## License

See LICENSE file in the project root.

## Support

For issues related to:
- **ROS Integration**: Check ROS logs and topic availability
- **Django Errors**: Review `django_server.log`
- **Recording Issues**: See `RECORDING_FEATURE.md` for detailed documentation

## Acknowledgments

- Built on Django web framework
- Uses rosbridge_suite for ROS communication
- Visualization powered by D3.js, Three.js, and Leaflet.js
- Part of the Earth Innovation Hub's Earth Rover project
