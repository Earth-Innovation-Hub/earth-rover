# GPS Map Visualizer for RTL-SDR ROS2

Real-time web-based GPS position visualization on an interactive map.

## Overview

The GPS Map Visualizer provides a web-based interface that displays GPS position in real-time on an interactive OpenStreetMap. It shows:

- **Live GPS Position**: Real-time marker showing current location
- **GPS Track**: Red line showing the path traveled
- **Position Info**: Latitude, longitude, altitude, HDOP, satellites, fix type
- **Status Indicators**: Visual status of GPS connection and fix quality

## Quick Start

### Launch GPS Reader with Map Visualization

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch GPS reader and map visualizer together
ros2 launch rtlsdr_ros2 gps_map_launch.py
```

### Access the Map

Open your web browser and navigate to:

```
http://localhost:8080
```

Or if accessing from another device on the network:

```
http://<your-computer-ip>:8080
```

## Features

### Interactive Map

- **OpenStreetMap Integration**: Uses OpenStreetMap tiles for detailed maps
- **Real-time Updates**: Position updates automatically via Server-Sent Events (SSE)
- **GPS Track**: Red polyline shows the path traveled
- **Auto-centering**: Map automatically centers on current GPS position
- **Zoom Controls**: Standard map zoom and pan controls

### Information Panel

The info panel displays:

- **Status**: Connection status and GPS fix type
- **Latitude/Longitude**: Current position coordinates
- **Altitude**: Height above sea level
- **HDOP**: Horizontal Dilution of Precision (accuracy indicator)
- **Satellites**: Number of satellites used (when available)
- **Fix Type**: 2D fix, 3D fix, or no fix
- **Updates**: Counter of position updates received

### Real-time Updates

- **Server-Sent Events (SSE)**: Primary method for real-time updates
- **HTTP Polling**: Fallback method every 2 seconds
- **Automatic Reconnection**: Handles connection drops gracefully

## Usage

### Basic Usage

```bash
# Launch with default settings (port 8080)
ros2 launch rtlsdr_ros2 gps_map_launch.py
```

### Custom Port

```bash
# Use a different port (e.g., 9000)
ros2 launch rtlsdr_ros2 gps_map_launch.py map_port:=9000
```

Then access at: `http://localhost:9000`

### Network Access

To access from other devices on your network:

```bash
# Bind to all interfaces (default)
ros2 launch rtlsdr_ros2 gps_map_launch.py map_host:=0.0.0.0
```

Then access from other devices using your computer's IP address:
```
http://192.168.1.100:8080  # Replace with your IP
```

### Standalone Map Visualizer

If GPS reader is already running:

```bash
# Run map visualizer separately
ros2 run rtlsdr_ros2 gps_map_visualizer
```

## Parameters

### GPS Map Visualizer Node

- `port` (int, default: 8080)
  - Port number for the web server
  
- `host` (string, default: '0.0.0.0')
  - Host address to bind to
  - '0.0.0.0' = all network interfaces
  - '127.0.0.1' = localhost only

### Launch File Parameters

- `map_port`: Port for web interface (default: 8080)
- `map_host`: Host address (default: '0.0.0.0')
- All GPS reader parameters are also available

## Topics

### Subscribed Topics

- **`/rtlsdr/gps_position`** (`sensor_msgs/NavSatFix`)
  - GPS position fixes
  - Must be published by GPS reader node

## Web Interface Endpoints

- **`/` or `/index.html`**
  - Main map interface (HTML page)

- **`/gps_data`**
  - Current GPS data as JSON
  - Returns 204 (No Content) if no GPS data available

- **`/gps_data_stream`**
  - Server-Sent Events stream
  - Real-time GPS position updates
  - Keeps connection open and sends updates as they arrive

## Example Workflow

1. **Start GPS Reader and Map**:
   ```bash
   ros2 launch rtlsdr_ros2 gps_map_launch.py
   ```

2. **Open Browser**:
   - Navigate to `http://localhost:8080`
   - You'll see the map with a default location

3. **Wait for GPS Fix**:
   - The map will update when GPS position is available
   - Status will change from "Waiting for GPS..." to "GPS Fix"

4. **Watch Position Update**:
   - Red marker shows current position
   - Red line shows track/path
   - Info panel updates with latest data

## Mobile Access

The web interface is mobile-friendly and can be accessed from smartphones/tablets:

1. Find your computer's IP address:
   ```bash
   hostname -I
   # or
   ip addr show
   ```

2. On your mobile device, open browser and go to:
   ```
   http://<your-ip>:8080
   ```

3. The map will work on mobile browsers with touch controls

## Troubleshooting

### Map Not Loading

- **Check if server is running**: Look for "Web server started" in logs
- **Check port**: Ensure port 8080 (or your custom port) is not in use
- **Check firewall**: Ensure port is not blocked by firewall
- **Try different browser**: Some browsers may have issues with SSE

### No GPS Position on Map

- **Check GPS reader**: Ensure GPS reader node is running and publishing
- **Check topic**: Verify `/rtlsdr/gps_position` topic has data:
  ```bash
  ros2 topic echo /rtlsdr/gps_position
  ```
- **Check connection**: Look at browser console for errors (F12)
- **Wait for fix**: GPS may take 30-60 seconds to acquire

### Connection Errors

- **SSE not supported**: Browser will fall back to polling
- **Network issues**: Check that host/port are correct
- **CORS issues**: Should not occur, but check browser console

## Advanced Usage

### Custom Map Tiles

You can modify the map visualizer code to use different map tile providers:

- **Mapbox**: Requires API key
- **Google Maps**: Requires API key
- **CartoDB**: Free alternative
- **Esri**: Various options

Edit the `get_map_html()` method in `gps_map_visualizer.py` to change the tile layer.

### Multiple GPS Sources

The visualizer subscribes to `/rtlsdr/gps_position`. You can remap this topic:

```bash
ros2 run rtlsdr_ros2 gps_map_visualizer --ros-args -r rtlsdr/gps_position:=/other/gps/topic
```

### Integration with RViz2

For 3D visualization, you can also use RViz2:

```bash
rviz2
# Add "Map" display and set topic to /rtlsdr/gps_position
```

## Technical Details

### Server-Sent Events (SSE)

The map uses SSE for real-time updates, which is:
- **One-way**: Server to client
- **HTTP-based**: Works through firewalls/proxies
- **Automatic reconnection**: Built-in reconnection handling
- **Efficient**: Lower overhead than WebSockets for one-way data

### Map Library

Uses **Leaflet.js** (loaded from CDN):
- Lightweight and fast
- Mobile-friendly
- Extensive plugin ecosystem
- Open source

### Map Data

Uses **OpenStreetMap** tiles:
- Free and open
- No API key required
- Good global coverage
- Community-maintained

## Related Documentation

- GPS Reader: `GPS_README.md`
- GPS Position: `GPS_POSITION_README.md`
- Main Package: `README.md`

## Notes

- The web server runs in a separate thread and doesn't block ROS2
- Map updates are throttled to prevent excessive updates
- GPS track is stored in browser memory (cleared on page refresh)
- No data is stored on the server - all processing is real-time

## Future Enhancements

Potential improvements:

1. **Track Export**: Export GPS track as GPX/KML
2. **Waypoints**: Add and manage waypoints
3. **Geofencing**: Set boundaries and alerts
4. **History**: Store and replay GPS tracks
5. **Multiple Devices**: Show multiple GPS sources
6. **Satellite View**: Toggle satellite imagery
7. **Offline Maps**: Support for offline map tiles

