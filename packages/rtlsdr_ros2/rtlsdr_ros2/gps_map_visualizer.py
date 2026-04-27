#!/usr/bin/env python3
"""
GPS Map Visualizer Node for RTL-SDR ROS2 Package
Provides a web-based real-time map visualization of GPS position.
"""

import sys
import os

# Add package path for imports when running as script
_script_dir = os.path.dirname(os.path.abspath(__file__))
_package_base = os.path.dirname(os.path.dirname(_script_dir))
_site_packages = os.path.join(_package_base, 'lib', 'python3.10', 'site-packages')
if os.path.exists(_site_packages) and _site_packages not in sys.path:
    sys.path.insert(0, _site_packages)
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import threading
import json
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import socketserver


class GpsMapHandler(BaseHTTPRequestHandler):
    """HTTP handler for serving the map interface and GPS data."""
    
    # Class variable to store GPS data reference
    gps_data_ref = None
    
    @classmethod
    def set_gps_data_ref(cls, gps_data_ref):
        """Set the GPS data reference."""
        cls.gps_data_ref = gps_data_ref
    
    @property
    def gps_data(self):
        """Get current GPS data."""
        if self.gps_data_ref:
            return self.gps_data_ref()
        return None
    
    def log_message(self, format, *args):
        """Suppress default logging."""
        pass
    
    def do_GET(self):
        """Handle GET requests."""
        parsed_path = urlparse(self.path)
        
        if parsed_path.path == '/' or parsed_path.path == '/index.html':
            self.serve_map_page()
        elif parsed_path.path == '/gps_data':
            self.serve_gps_data()
        elif parsed_path.path == '/gps_data_stream':
            self.serve_gps_data_stream()
        else:
            self.send_error(404)
    
    def serve_map_page(self):
        """Serve the main map HTML page."""
        html_content = self.get_map_html()
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Content-length', str(len(html_content)))
        self.end_headers()
        self.wfile.write(html_content.encode())
    
    def serve_gps_data(self):
        """Serve current GPS data as JSON."""
        gps_data = self.gps_data
        if gps_data:
            try:
                data = json.dumps(gps_data).encode()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                self.send_error(500, f"Error encoding GPS data: {str(e)}")
        else:
            # Return empty JSON object instead of 204
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-length', '2')
            self.end_headers()
            self.wfile.write(b'{}')
    
    def serve_gps_data_stream(self):
        """Serve GPS data as Server-Sent Events (SSE) stream."""
        self.send_response(200)
        self.send_header('Content-type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        
        # Send initial connection message
        self.wfile.write(b'data: {"status": "connected"}\n\n')
        self.wfile.flush()
        
        # Keep connection open and send updates
        last_data = None
        while True:
            try:
                gps_data = self.gps_data
                if gps_data and gps_data != last_data:
                    try:
                        data_json = json.dumps(gps_data)
                        self.wfile.write(f'data: {data_json}\n\n'.encode())
                        self.wfile.flush()
                        last_data = gps_data
                    except Exception as e:
                        # Send error in SSE format
                        error_msg = json.dumps({"error": str(e)})
                        self.wfile.write(f'data: {error_msg}\n\n'.encode())
                        self.wfile.flush()
                else:
                    # Send keepalive
                    self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
                time.sleep(0.5)  # Update every 500ms
            except (BrokenPipeError, ConnectionResetError, OSError):
                break
            except Exception as e:
                # Log error but keep connection alive
                try:
                    error_msg = json.dumps({"error": str(e)})
                    self.wfile.write(f'data: {error_msg}\n\n'.encode())
                    self.wfile.flush()
                except:
                    break
                time.sleep(1)
    
    def get_map_html(self):
        """Generate HTML page with interactive map."""
        return """<!DOCTYPE html>
<html>
<head>
    <title>RTL-SDR GPS Position Map</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        body {
            margin: 0;
            padding: 0;
            font-family: Arial, sans-serif;
        }
        #map {
            height: 100vh;
            width: 100%;
        }
        #info {
            position: absolute;
            top: 10px;
            left: 10px;
            background: white;
            padding: 15px;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.3);
            z-index: 1000;
            min-width: 250px;
        }
        #status {
            font-weight: bold;
            margin-bottom: 10px;
        }
        .status-connected { color: green; }
        .status-disconnected { color: red; }
        .status-waiting { color: orange; }
        .info-row {
            margin: 5px 0;
            font-size: 14px;
        }
        .label {
            font-weight: bold;
            display: inline-block;
            width: 100px;
        }
    </style>
</head>
<body>
    <div id="map"></div>
    <div id="info">
        <div id="status" class="status-waiting">● Waiting for GPS...</div>
        <div class="info-row"><span class="label">Latitude:</span> <span id="lat">--</span></div>
        <div class="info-row"><span class="label">Longitude:</span> <span id="lon">--</span></div>
        <div class="info-row"><span class="label">Altitude:</span> <span id="alt">--</span> m</div>
        <div class="info-row"><span class="label">HDOP:</span> <span id="hdop">--</span></div>
        <div class="info-row"><span class="label">Satellites:</span> <span id="sats">--</span></div>
        <div class="info-row"><span class="label">Fix Type:</span> <span id="fix">--</span></div>
        <div class="info-row"><span class="label">Updates:</span> <span id="updates">0</span></div>
    </div>

    <script>
        // Initialize map
        var map = L.map('map').setView([37.7749, -122.4194], 13); // Default to San Francisco
        
        // Add OpenStreetMap tile layer
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19
        }).addTo(map);
        
        // GPS marker and track
        var gpsMarker = null;
        var trackPolyline = null;
        var trackPoints = [];
        var updateCount = 0;
        
        // Update GPS position
        function updateGPS(data) {
            if (!data || !data.latitude || !data.longitude) {
                return;
            }
            
            var lat = data.latitude;
            var lon = data.longitude;
            var alt = data.altitude || 0;
            var hdop = data.hdop || 0;
            var satellites = data.satellites_used || 0;
            var fixType = data.fix_type || 0;
            
            // Update info panel
            document.getElementById('lat').textContent = lat.toFixed(6) + '°';
            document.getElementById('lon').textContent = lon.toFixed(6) + '°';
            document.getElementById('alt').textContent = alt.toFixed(1);
            document.getElementById('hdop').textContent = hdop.toFixed(2);
            document.getElementById('sats').textContent = satellites;
            document.getElementById('fix').textContent = getFixTypeName(fixType);
            document.getElementById('updates').textContent = ++updateCount;
            
            // Update status
            var statusEl = document.getElementById('status');
            if (fixType >= 3) {
                statusEl.textContent = '● GPS Fix (3D)';
                statusEl.className = 'status-connected';
            } else if (fixType == 2) {
                statusEl.textContent = '● GPS Fix (2D)';
                statusEl.className = 'status-connected';
            } else {
                statusEl.textContent = '● No Fix';
                statusEl.className = 'status-waiting';
            }
            
            // Update or create marker
            if (gpsMarker) {
                gpsMarker.setLatLng([lat, lon]);
            } else {
                var icon = L.icon({
                    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
                    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                    iconSize: [25, 41],
                    iconAnchor: [12, 41],
                    popupAnchor: [1, -34],
                    shadowSize: [41, 41]
                });
                gpsMarker = L.marker([lat, lon], {icon: icon}).addTo(map);
                gpsMarker.bindPopup('GPS Position<br>Lat: ' + lat.toFixed(6) + '<br>Lon: ' + lon.toFixed(6));
            }
            
            // Add to track
            trackPoints.push([lat, lon]);
            
            // Update track polyline
            if (trackPolyline) {
                trackPolyline.setLatLngs(trackPoints);
            } else {
                trackPolyline = L.polyline(trackPoints, {
                    color: 'red',
                    weight: 3,
                    opacity: 0.7
                }).addTo(map);
            }
            
            // Center map on position (with smooth transition)
            map.setView([lat, lon], map.getZoom(), {animate: true, duration: 0.5});
        }
        
        function getFixTypeName(fixType) {
            switch(fixType) {
                case 0: return 'No Fix';
                case 1: return 'No Fix';
                case 2: return '2D Fix';
                case 3: return '3D Fix';
                default: return 'Unknown';
            }
        }
        
        // Connect to GPS data stream
        var eventSource = new EventSource('/gps_data_stream');
        
        eventSource.onmessage = function(event) {
            try {
                var data = JSON.parse(event.data);
                if (data.latitude && data.longitude) {
                    updateGPS(data);
                }
            } catch (e) {
                console.error('Error parsing GPS data:', e);
            }
        };
        
        eventSource.onerror = function(event) {
            console.error('EventSource error:', event);
            document.getElementById('status').textContent = '● Connection Error';
            document.getElementById('status').className = 'status-disconnected';
        };
        
        // Also try polling as fallback
        function pollGPS() {
            fetch('/gps_data')
                .then(response => response.json())
                .then(data => {
                    if (data && data.latitude && data.longitude) {
                        updateGPS(data);
                    }
                })
                .catch(err => console.error('Polling error:', err));
        }
        
        // Poll every 2 seconds as backup
        setInterval(pollGPS, 2000);
    </script>
</body>
</html>"""


class GpsMapVisualizerNode(Node):
    """ROS2 Node that visualizes GPS position on a web-based map."""

    def __init__(self):
        super().__init__('gps_map_visualizer')

        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value

        # GPS data storage
        self.gps_data = None
        self.last_update_time = None

        # Create subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            'rtlsdr/gps_position',
            self.gps_callback,
            10
        )

        # Start web server in separate thread
        self.server_thread = None
        self.httpd = None
        self.start_web_server()

        self.get_logger().info('GPS Map Visualizer Node started')
        self.get_logger().info(f'Web interface available at: http://localhost:{self.port}')
        self.get_logger().info(f'Subscribing to: /rtlsdr/gps_position')

    def gps_callback(self, msg: NavSatFix):
        """Process incoming GPS position messages."""
        # Extract position data
        self.gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'hdop': self.calculate_hdop_from_covariance(msg.position_covariance),
            'vdop': self.calculate_vdop_from_covariance(msg.position_covariance),
            'fix_type': self.get_fix_type_from_status(msg.status.status),
            'satellites_used': 0,  # Not available in NavSatFix, would need custom message
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        self.last_update_time = time.time()

        # Log position updates
        self.get_logger().debug(
            f'GPS Position: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.1f}m'
        )

    def calculate_hdop_from_covariance(self, covariance):
        """Estimate HDOP from position covariance."""
        if len(covariance) >= 4:
            # HDOP ≈ sqrt(covariance[0] + covariance[4]) / 5.0
            h_variance = (covariance[0] + covariance[4]) / 2.0
            return (h_variance ** 0.5) / 5.0
        return 0.0

    def calculate_vdop_from_covariance(self, covariance):
        """Estimate VDOP from position covariance."""
        if len(covariance) >= 9:
            # VDOP ≈ sqrt(covariance[8]) / 5.0
            v_variance = covariance[8]
            return (v_variance ** 0.5) / 5.0
        return 0.0

    def get_fix_type_from_status(self, status):
        """Convert NavSatStatus to fix type."""
        if status == NavSatFix.STATUS_FIX:
            return 3  # 3D fix
        elif status == NavSatFix.STATUS_SBAS_FIX:
            return 2  # 2D fix
        else:
            return 0  # No fix

    def start_web_server(self):
        """Start the web server in a separate thread."""
        def run_server():
            # Set the GPS data reference in the handler class
            GpsMapHandler.set_gps_data_ref(lambda: self.gps_data)
            
            self.httpd = HTTPServer((self.host, self.port), GpsMapHandler)
            self.get_logger().info(f'Web server started on http://{self.host}:{self.port}')
            self.get_logger().info(f'Access the map at: http://localhost:{self.port}')
            if self.host == '0.0.0.0':
                import socket
                try:
                    hostname = socket.gethostname()
                    local_ip = socket.gethostbyname(hostname)
                    self.get_logger().info(f'Or from network: http://{local_ip}:{self.port}')
                except:
                    pass
            try:
                self.httpd.serve_forever()
            except Exception as e:
                self.get_logger().error(f'Web server error: {str(e)}')

        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()

    def shutdown(self):
        """Shutdown the node and web server."""
        if self.httpd:
            try:
                # Shutdown the server
                self.httpd.shutdown()
                # Give it a moment to close (but don't block too long)
                import time
                time.sleep(0.1)
            except (KeyboardInterrupt, Exception) as e:
                # Ignore errors during shutdown, especially KeyboardInterrupt
                pass
        try:
            self.get_logger().info('GPS Map Visualizer Node shutdown')
        except:
            pass  # Logger may be invalid during shutdown


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = None

    try:
        node = GpsMapVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if node is not None:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception as e:
                print(f'Error during node cleanup: {str(e)}')
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Already shut down


if __name__ == '__main__':
    main()

