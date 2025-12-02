#!/usr/bin/env python3
"""
DeepGIS GPS Telemetry Publisher Node

Subscribes to MAVROS GPS topics and publishes GPS data to DeepGIS API
in compliance with the DeepGIS Telemetry API specification.
Only sends GPS signals, no batching - sends immediately.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32

import requests
import json
import math
from datetime import datetime
from threading import Lock


class DeepGISGPSPublisher(Node):
    """
    ROS2 Node that publishes GPS telemetry data to DeepGIS API.
    Only sends GPS signals, no batching - sends immediately.
    """

    def __init__(self):
        super().__init__('deepgis_gps_publisher')

        # Declare parameters
        self.declare_parameter('deepgis_api_url', 'https://deepgis.org')
        self.declare_parameter('api_key', 'test-missions')
        self.declare_parameter('asset_name', 'EarthRover')
        self.declare_parameter('session_id', '')  # Auto-generated if empty
        self.declare_parameter('project_title', 'EarthRover: Affordable and Sustainable Mobility Autonomy for  4D Environmental Monitoring')
        self.declare_parameter('flight_mode', 'AUTO')
        self.declare_parameter('mission_type', 'Telemetry Collection')
        self.declare_parameter('notes', 'Automated GPS telemetry collection from MAVROS')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('publish_rate', 0.1)  # Default: 0.1 Hz = once every 10 seconds
        
        # Get parameters
        self.api_url = self.get_parameter('deepgis_api_url').value
        self.api_key = self.get_parameter('api_key').value
        self.asset_name = self.get_parameter('asset_name').value
        session_id_param = self.get_parameter('session_id').value
        self.session_id = session_id_param if session_id_param else \
                         f"mavros_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.project_title = self.get_parameter('project_title').value
        self.flight_mode = self.get_parameter('flight_mode').value
        self.mission_type = self.get_parameter('mission_type').value
        self.notes = self.get_parameter('notes').value
        self.mavros_ns = self.get_parameter('mavros_namespace').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # API endpoints
        self.api_endpoints = {
            'create_session': f'{self.api_url}/api/telemetry/session/create/',
            'gps_raw': f'{self.api_url}/api/telemetry/gps-fix-raw/',
            'gps_estimated': f'{self.api_url}/api/telemetry/gps-fix-estimated/',
        }

        # Session management
        self.session_active = False

        # Satellite count
        self.satellites_visible = 0

        # Latest GPS data storage
        self.latest_gps_raw = None
        self.latest_gps_estimated = None
        self.data_lock = Lock()

        # HTTP session with connection pooling
        self.http_session = requests.Session()
        if self.api_key:
            self.http_session.headers.update({'Authorization': f'Bearer {self.api_key}'})
        self.http_session.headers.update({'Content-Type': 'application/json'})

        # QoS profile for MAVROS topics (best effort, like MAVROS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers - only GPS topics
        self.gps_raw_sub = self.create_subscription(
            NavSatFix,
            f'{self.mavros_ns}/global_position/raw/fix',
            self.gps_raw_callback,
            qos_profile
        )

        self.gps_estimated_sub = self.create_subscription(
            NavSatFix,
            f'{self.mavros_ns}/global_position/global',
            self.gps_estimated_callback,
            qos_profile
        )
        
        self.satellites_sub = self.create_subscription(
            UInt32,
            f'{self.mavros_ns}/global_position/raw/satellites',
            self.satellites_callback,
            qos_profile
        )

        # Create telemetry session
        self.create_telemetry_session()

        # Publisher timer - publishes at specified rate
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_telemetry
        )

        self.get_logger().info('DeepGIS GPS Publisher initialized')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Asset Name: {self.asset_name}')
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Publish Rate: {self.publish_rate} Hz (every {1.0/self.publish_rate:.1f} seconds)')
        self.get_logger().info('Mode: GPS-only (no batching)')

    def log_response(self, response, endpoint_name, request_url=None):
        """
        Log server response (success or error) from DeepGIS server.
        
        Args:
            response: requests.Response object
            endpoint_name: Name of the endpoint for logging context
            request_url: Optional URL that was requested
        """
        url = request_url or (response.url if hasattr(response, 'url') else 'unknown')
        is_success = response.status_code in [200, 201]
        
        if is_success:
            msg = f'[{endpoint_name}] Request successful'
        else:
            msg = f'[{endpoint_name}] Request failed'
        
        msg += f'\n  URL: {url}'
        msg += f'\n  Status Code: {response.status_code}'
        msg += f'\n  Status Text: {response.reason if hasattr(response, "reason") else "N/A"}'
        
        # Try to parse JSON response
        try:
            response_json = response.json()
            msg += f'\n  Response JSON: {json.dumps(response_json, indent=2)}'
        except (ValueError, json.JSONDecodeError):
            # Not JSON, show raw text
            response_text = response.text[:500]  # Limit to 500 chars
            msg += f'\n  Response Text: {response_text}'
            if len(response.text) > 500:
                msg += '... (truncated)'
        
        # Log response headers if available
        if hasattr(response, 'headers') and response.headers:
            msg += f'\n  Response Headers: {dict(response.headers)}'
        
        if is_success:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)
    
    def log_error_response(self, response, endpoint_name, request_url=None):
        """
        Log detailed error response from DeepGIS server.
        (Kept for backward compatibility, now calls log_response)
        """
        self.log_response(response, endpoint_name, request_url)

    def create_telemetry_session(self):
        """Create a new telemetry session with DeepGIS API (API compliant format)."""
        try:
            payload = {
                'session_id': self.session_id,
                'asset_name': self.asset_name,
                'project_title': self.project_title,
                'flight_mode': self.flight_mode,
                'mission_type': self.mission_type,
                'notes': self.notes
            }

            response = self.http_session.post(
                self.api_endpoints['create_session'],
                json=payload,
                timeout=10.0
            )

            if response.status_code in [200, 201]:
                self.session_active = True
                self.get_logger().info(f'Created telemetry session: {self.session_id}')
            
            # Log server response (success or error)
            self.log_response(response, 'create_session', self.api_endpoints['create_session'])

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error creating telemetry session: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'create_session', self.api_endpoints['create_session'])
        except Exception as e:
            self.get_logger().error(f'Unexpected error creating session: {str(e)}')

    def satellites_callback(self, msg: UInt32):
        """Callback for satellites visible."""
        self.satellites_visible = msg.data

    def publish_telemetry(self):
        """Publish telemetry data to DeepGIS API at the configured rate."""
        if not self.session_active:
            return

        # Get latest data references quickly (minimal lock time)
        with self.data_lock:
            gps_raw = self.latest_gps_raw
            gps_estimated = self.latest_gps_estimated

        # Format and send GPS data
        if gps_raw:
            formatted_data = self.format_gps_data(gps_raw, is_raw=True)
            self.send_gps_raw(formatted_data)
        
        if gps_estimated:
            formatted_data = self.format_gps_data(gps_estimated, is_raw=False)
            self.send_gps_estimated(formatted_data)

    def gps_raw_callback(self, msg: NavSatFix):
        """Callback for raw GPS fix data - stores latest data."""
        with self.data_lock:
            self.latest_gps_raw = msg

    def gps_estimated_callback(self, msg: NavSatFix):
        """Callback for estimated GPS position - stores latest data."""
        with self.data_lock:
            self.latest_gps_estimated = msg

    def extract_gps_accuracy(self, msg: NavSatFix):
        """
        Extract horizontal and vertical accuracy from GPS covariance.
        
        Returns:
            (eph, epv) in meters
        """
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return 99.99, 99.99
        
        # Extract variances (diagonal elements)
        var_lat = msg.position_covariance[0]  # latitude variance
        var_lon = msg.position_covariance[4]  # longitude variance
        var_alt = msg.position_covariance[8]  # altitude variance
        
        # Convert to standard deviations
        # eph is approximately the average of lat/lon std devs
        eph = math.sqrt((var_lat + var_lon) / 2.0)
        epv = math.sqrt(var_alt)
        
        return float(eph), float(epv)

    def map_gps_fix_type(self, status):
        """
        Map NavSatStatus to GPS fix type.
        
        STATUS_NO_FIX = -1
        STATUS_FIX = 0
        STATUS_SBAS_FIX = 1
        STATUS_GBAS_FIX = 2
        
        Returns fix type: 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GPS+DR, 5=Time only
        """
        if status < 0:
            return 0  # No fix
        elif status == 0:
            return 3  # 3D fix
        elif status == 1:
            return 4  # SBAS (augmented, treat as GPS+DR)
        elif status == 2:
            return 4  # GBAS (ground-based augmentation, treat as GPS+DR)
        else:
            return 3  # Default to 3D
    
    def format_gps_data(self, msg: NavSatFix, is_raw=True):
        """Format GPS fix data for DeepGIS API (compliant format)."""
        # Extract accuracy
        eph, epv = self.extract_gps_accuracy(msg)
        
        # Map fix type
        fix_type = self.map_gps_fix_type(msg.status.status)
        
        # Convert ROS timestamp to microseconds
        timestamp_usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        
        result = {
            'session_id': self.session_id,
            'timestamp': datetime.now().isoformat(),
            'timestamp_usec': timestamp_usec,
            'latitude': float(msg.latitude),
            'longitude': float(msg.longitude),
            'altitude': float(msg.altitude),
            'fix_type': fix_type,
            'eph': float(eph),
            'epv': float(epv),
            'satellites_visible': int(self.satellites_visible)
        }
        
        return result

    def send_gps_raw(self, data):
        """Send raw GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_raw'],
                json=data,
                timeout=5.0
            )
            # Log server response (success or error)
            self.log_response(response, 'gps_raw', self.api_endpoints['gps_raw'])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS raw: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_response(e.response, 'gps_raw', self.api_endpoints['gps_raw'])

    def send_gps_estimated(self, data):
        """Send estimated GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_estimated'],
                json=data,
                timeout=5.0
            )
            # Log server response (success or error)
            self.log_response(response, 'gps_estimated', self.api_endpoints['gps_estimated'])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS estimated: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_response(e.response, 'gps_estimated', self.api_endpoints['gps_estimated'])

    def destroy_node(self):
        """Cleanup before node shutdown."""
        # Close HTTP session
        try:
            self.http_session.close()
        except Exception:
            pass  # Ignore errors during shutdown
        
        try:
            super().destroy_node()
        except Exception:
            pass  # Ignore errors if node already destroyed


def main(args=None):
    rclpy.init(args=args)
    
    node = DeepGISGPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

