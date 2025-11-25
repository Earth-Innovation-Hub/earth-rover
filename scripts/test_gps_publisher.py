#!/usr/bin/env python3
"""
Test GPS Publisher - Simplified version for debugging
Only publishes GPS raw fix data with detailed logging
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


class TestGPSPublisher(Node):
    """Simplified GPS publisher for testing."""

    def __init__(self):
        super().__init__('test_gps_publisher')

        # Declare parameters
        self.declare_parameter('deepgis_api_url', 'https://deepgis.org')
        self.declare_parameter('api_key', '')
        self.declare_parameter('asset_name', 'Test Vehicle')
        self.declare_parameter('session_id', f"test_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        
        # Get parameters
        self.api_url = self.get_parameter('deepgis_api_url').value
        self.api_key = self.get_parameter('api_key').value
        self.asset_name = self.get_parameter('asset_name').value
        self.session_id = self.get_parameter('session_id').value

        # API endpoints
        self.create_session_url = f'{self.api_url}/api/telemetry/session/create/'
        self.gps_raw_url = f'{self.api_url}/api/telemetry/gps-fix-raw/'

        # Session management
        self.session_active = False
        self.gps_count = 0
        self.satellites_visible = 0

        # HTTP session
        self.http_session = requests.Session()
        if self.api_key:
            self.http_session.headers.update({'Authorization': f'Bearer {self.api_key}'})
        self.http_session.headers.update({'Content-Type': 'application/json'})

        # QoS profile for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile
        )
        
        self.satellites_sub = self.create_subscription(
            UInt32,
            '/mavros/global_position/raw/satellites',
            self.satellites_callback,
            qos_profile
        )

        self.get_logger().info('=== Test GPS Publisher Started ===')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Asset Name: {self.asset_name}')
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Subscribed to: /mavros/global_position/raw/fix')

        # Create session
        self.create_telemetry_session()

    def create_telemetry_session(self):
        """Create a new telemetry session."""
        try:
            payload = {
                'session_id': self.session_id,
                'asset_name': self.asset_name,
                'project_title': 'Test Mission',
                'flight_mode': 'AUTO',
                'mission_type': 'Testing',
                'notes': 'Test GPS data publishing'
            }

            self.get_logger().info(f'Creating session with payload: {json.dumps(payload, indent=2)}')

            response = self.http_session.post(
                self.create_session_url,
                json=payload,
                timeout=10.0
            )

            self.get_logger().info(f'Session creation response: {response.status_code}')
            self.get_logger().info(f'Response body: {response.text}')

            if response.status_code in [200, 201]:
                self.session_active = True
                self.get_logger().info(f'✅ Session created successfully: {self.session_id}')
            else:
                self.get_logger().error(f'❌ Failed to create session: {response.status_code} - {response.text}')

        except Exception as e:
            self.get_logger().error(f'❌ Error creating session: {str(e)}')

    def satellites_callback(self, msg: UInt32):
        """Callback for satellites visible."""
        self.satellites_visible = msg.data

    def gps_callback(self, msg: NavSatFix):
        """Callback for GPS data."""
        self.gps_count += 1
        
        if self.gps_count % 5 == 1:  # Log every 5th message
            self.get_logger().info(f'📡 Received GPS #{self.gps_count}: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}, sats={self.satellites_visible}')
        
        if not self.session_active:
            if self.gps_count == 1:
                self.get_logger().warn('⚠️  Session not active, skipping GPS publish')
            return

        # Format and send
        try:
            gps_data = self.format_gps_data(msg)
            self.send_gps_raw(gps_data)
        except Exception as e:
            self.get_logger().error(f'❌ Error in GPS callback: {str(e)}')

    def format_gps_data(self, msg: NavSatFix):
        """Format GPS data."""
        # Extract accuracy
        eph, epv = self.extract_gps_accuracy(msg)
        
        # Map fix type
        fix_type = self.map_gps_fix_type(msg.status.status)
        
        # Convert timestamp
        timestamp_usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        
        data = {
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
        
        if self.gps_count == 1:
            self.get_logger().info(f'📤 First GPS payload: {json.dumps(data, indent=2)}')
        
        return data

    def extract_gps_accuracy(self, msg: NavSatFix):
        """Extract GPS accuracy."""
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return 99.99, 99.99
        
        var_lat = msg.position_covariance[0]
        var_lon = msg.position_covariance[4]
        var_alt = msg.position_covariance[8]
        
        eph = math.sqrt((var_lat + var_lon) / 2.0)
        epv = math.sqrt(var_alt)
        
        return float(eph), float(epv)

    def map_gps_fix_type(self, status):
        """Map GPS fix type."""
        if status < 0:
            return 0  # No fix
        elif status == 0:
            return 3  # 3D fix
        elif status == 1:
            return 4  # SBAS
        elif status == 2:
            return 6  # GBAS/RTK
        else:
            return 3

    def send_gps_raw(self, data):
        """Send GPS data to API."""
        try:
            response = self.http_session.post(
                self.gps_raw_url,
                json=data,
                timeout=5.0
            )
            
            if response.status_code in [200, 201]:
                if self.gps_count % 10 == 0:  # Log every 10th successful send
                    self.get_logger().info(f'✅ GPS #{self.gps_count} sent successfully')
            else:
                self.get_logger().error(f'❌ Failed to send GPS #{self.gps_count}: {response.status_code}')
                self.get_logger().error(f'Response: {response.text}')
                
        except requests.exceptions.Timeout:
            self.get_logger().error(f'❌ Timeout sending GPS #{self.gps_count}')
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f'❌ Connection error sending GPS #{self.gps_count}: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'❌ Error sending GPS #{self.gps_count}: {str(e)}')

    def destroy_node(self):
        """Cleanup."""
        self.get_logger().info(f'=== Test GPS Publisher Stopped (sent {self.gps_count} GPS messages) ===')
        self.http_session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = TestGPSPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

