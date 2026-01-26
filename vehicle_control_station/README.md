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
- **Automatic ROS Version Detection**: Compatible with both ROS1 and ROS2
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
- **Backend**: Django 3.x (Python)
- **Frontend**: 
  - HTML5/CSS3/JavaScript
  - D3.js for data visualization
  - Three.js for 3D rendering
  - Leaflet.js for mapping
  - ROSLib.js for ROS communication
- **Communication**: WebSocket (rosbridge)
- **Video Streaming**: web-mjpeg-server

### API Endpoints

#### `/api/disk-space/` (GET)
Returns current disk space information for rosbag storage:
```json
{
  "total_gb": 291.0,
  "used_gb": 267.0,
  "free_gb": 9.1,
  "used_percent": 97.0,
  "status": "ok"
}
```

#### `/api/recording/start/` (POST)
Starts recording all ROS topics:
```json
{
  "status": "ok",
  "message": "Started recording all topics (ROS2)",
  "recording": true,
  "output_file": "/home/user/rosbags/rosbag2_2026_01_25-18_30_45",
  "pid": 12345
}
```

#### `/api/recording/stop/` (POST)
Stops the current recording:
```json
{
  "status": "ok",
  "message": "Recording stopped successfully",
  "recording": false,
  "output_file": "/home/user/rosbags/rosbag2_2026_01_25-18_30_45"
}
```

#### `/api/recording/status/` (GET)
Returns current recording status:
```json
{
  "status": "ok",
  "recording": true,
  "output_file": "/home/user/rosbags/rosbag2_2026_01_25-18_30_45",
  "pid": 12345
}
```

## Prerequisites

### System Requirements
- Python 3.8+
- ROS1 (Noetic) or ROS2 (Foxy/Humble/Rolling)
- rosbridge_suite
- web_video_server or similar MJPEG streamer

### ROS Topics Expected
The application subscribes to the following topics:
- `/stereo/left/image_raw` - Left stereo camera
- `/stereo/right/image_raw` - Right stereo camera
- `/camera/fisheye1/image_raw` - Fisheye camera
- `/mavros/global_position/raw/fix` - GPS data
- `/scan` - 2D laser scan
- `/velodyne_points` - Velodyne point cloud
- `/camera/pose/sample` - Visual odometry
- `/spectrometer` - Spectrometer data
- `/serial_data` - Laser ranger data

## Installation

### 1. Install Dependencies

Using Poetry (recommended):
```bash
cd vehicle_control_station
poetry install
```

Or using pip:
```bash
pip install Django>=3.0
```

### 2. Configure Django

Set the Django secret key:
```bash
export SECRET_KEY=$(python -c "import secrets; print(secrets.token_urlsafe(32))")
```

Or create a `.env` file:
```bash
SECRET_KEY=your-secret-key-here
```

### 3. Run Migrations
```bash
python manage.py migrate
```

### 4. Create Static Files
```bash
python manage.py collectstatic --noinput
```

## Usage

### Starting the Server

#### Development Mode
```bash
python manage.py runserver 0.0.0.0:8000
```

#### Production Mode
```bash
# Using gunicorn
gunicorn vehicle_control_station.wsgi:application --bind 0.0.0.0:8000

# Or run in background
nohup python manage.py runserver 0.0.0.0:8000 > django_server.log 2>&1 &
```

### Starting Required ROS Services

1. **Start rosbridge** (for WebSocket communication):
```bash
# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. **Start web video server** (for camera feeds):
```bash
# ROS2
ros2 run web_video_server web_video_server

# ROS1
rosrun web_video_server web_video_server
```

### Accessing the Interface

Navigate to: `http://localhost:8000` or `http://<your-ip>:8000`

### Recording ROS Topics

1. Open the web interface
2. Check the "Rosbag Storage Capacity" widget to ensure sufficient disk space
3. Click "Start Recording" to begin recording all topics
4. The status indicator will show "Recording" with a blinking red dot
5. Click "Stop Recording" when finished
6. Rosbags are saved to `~/rosbags/` with timestamp format: `rosbag2_YYYY_MM_DD-HH_MM_SS`

## Configuration

### WebSocket Connection
Edit the ROS bridge URL in `templates/home.html`:
```javascript
var ros = new ROSLIB.Ros({ url: 'ws://192.168.0.35:9090' });
```

### Camera Stream URLs
Modify camera stream sources in `templates/home.html`:
```html
<img src="http://192.168.0.35:8080/stream?topic=/stereo/left/image_raw">
```

### Recording Output Directory
Default: `~/rosbags/`

To change, modify the `start_recording` function in `vehicle_control_station/views.py`:
```python
rosbag_dir = os.path.expanduser('~/your-custom-path')
```

## Project Structure

```
vehicle_control_station/
├── manage.py                      # Django management script
├── db.sqlite3                     # SQLite database
├── README.md                      # This file
├── RECORDING_FEATURE.md           # Detailed recording feature docs
├── pyproject.toml                 # Poetry dependencies
├── templates/
│   └── home.html                  # Main dashboard interface
├── staticfiles/                   # Collected static files
└── vehicle_control_station/
    ├── settings.py                # Django settings
    ├── urls.py                    # URL routing
    ├── views.py                   # API views and logic
    ├── models.py                  # Database models
    └── static/
        └── vehicle-control-station/
            └── roslib.min.js      # ROS JavaScript library
```

## Troubleshooting

### WebSocket Connection Issues
- Ensure rosbridge is running: `rosnode list | grep rosbridge`
- Check firewall rules allow connections on port 9090
- Verify the WebSocket URL matches your ROS master IP

### Camera Feeds Not Showing
- Confirm web_video_server is running
- Verify camera topics are publishing: `ros2 topic list | grep image` (ROS2) or `rostopic list | grep image` (ROS1)
- Check the camera stream URLs match your setup

### Recording Fails to Start
- Verify ROS environment is sourced
- Check disk space availability
- Ensure `~/rosbags/` directory is writable
- Check ROS version compatibility (ROS1 vs ROS2)

### Point Cloud Not Rendering
- Verify Velodyne frame_id is set to "velodyne" in the driver launch file
- Check `/velodyne_points` topic is publishing: `ros2 topic echo /velodyne_points --once`
- Ensure point cloud contains valid data

## Security Considerations

- **CSRF Protection**: Enabled for all POST requests
- **Process Isolation**: Recording processes use separate process groups
- **Input Validation**: API endpoints validate request data
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
