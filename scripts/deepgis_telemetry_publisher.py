#!/usr/bin/env python3
"""
DeepGIS Telemetry Publisher Node

Subscribes to MAVROS telemetry topics and publishes data to DeepGIS API
in compliance with the DeepGIS Telemetry API specification.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt32

import requests
import json
import math
from datetime import datetime
from threading import Lock
import time


class DeepGISTelemetryPublisher(Node):
    """
    ROS2 Node that publishes vehicle telemetry data to DeepGIS API.
    Fully compliant with DeepGIS Telemetry API specification.
    """

    def __init__(self):
        super().__init__('deepgis_telemetry_publisher')

        # Declare parameters
        self.declare_parameter('deepgis_api_url', 'https://deepgis.org')
        self.declare_parameter('api_key', '')
        self.declare_parameter('asset_name', 'MAVROS Vehicle')
        self.declare_parameter('session_id', '')  # Auto-generated if empty
        self.declare_parameter('project_title', 'MAVROS Data Collection')
        self.declare_parameter('flight_mode', 'AUTO')
        self.declare_parameter('mission_type', 'Telemetry Collection')
        self.declare_parameter('notes', 'Automated telemetry collection from MAVROS')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('batch_size', 10)
        self.declare_parameter('enable_batch_mode', True)
        
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
        self.batch_size = self.get_parameter('batch_size').value
        self.enable_batch = self.get_parameter('enable_batch_mode').value

        # API endpoints
        self.api_endpoints = {
            'create_session': f'{self.api_url}/api/telemetry/session/create/',
            'local_position': f'{self.api_url}/api/telemetry/local-position-odom/',
            'gps_raw': f'{self.api_url}/api/telemetry/gps-fix-raw/',
            'gps_estimated': f'{self.api_url}/api/telemetry/gps-fix-estimated/',
            'batch': f'{self.api_url}/api/telemetry/batch/',
        }

        # Session management
        self.session_active = False

        # Reference position (set from first valid GPS fix)
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.reference_set = False
        
        # Satellite count
        self.satellites_visible = 0

        # Data buffers for batch mode
        self.local_position_buffer = []
        self.gps_raw_buffer = []
        self.gps_estimated_buffer = []
        self.buffer_lock = Lock()

        # Latest data storage
        self.latest_odom = None
        self.latest_gps_raw = None
        self.latest_gps_estimated = None
        self.data_lock = Lock()

        # QoS profile for MAVROS topics (best effort, like MAVROS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'{self.mavros_ns}/local_position/odom',
            self.odom_callback,
            qos_profile
        )

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

        # HTTP session with connection pooling
        self.http_session = requests.Session()
        if self.api_key:
            self.http_session.headers.update({'Authorization': f'Bearer {self.api_key}'})
        self.http_session.headers.update({'Content-Type': 'application/json'})

        # Create telemetry session
        self.create_telemetry_session()

        # Publisher timer
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_telemetry
        )

        self.get_logger().info('DeepGIS Telemetry Publisher initialized')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Asset Name: {self.asset_name}')
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Batch Mode: {self.enable_batch}')

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
            else:
                self.get_logger().error(
                    f'Failed to create session: {response.status_code} - {response.text}'
                )

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error creating telemetry session: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error creating session: {str(e)}')

    def quaternion_to_heading(self, x, y, z, w):
        """
        Convert quaternion to heading angle (yaw) in radians.
        
        Args:
            x, y, z, w: Quaternion components
            
        Returns:
            Heading angle in radians [-pi, pi]
        """
        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        return heading

    def set_reference_position(self, msg: NavSatFix):
        """Set reference position from first valid GPS fix."""
        if not self.reference_set and msg.status.status >= 0:
            # Check for valid GPS data
            if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                self.ref_lat = msg.latitude
                self.ref_lon = msg.longitude
                self.ref_alt = msg.altitude if not math.isnan(msg.altitude) else 0.0
                self.reference_set = True
                self.get_logger().info(
                    f'Set reference position: ({self.ref_lat:.6f}, '
                    f'{self.ref_lon:.6f}, {self.ref_alt:.2f}m)'
                )

    def satellites_callback(self, msg: UInt32):
        """Callback for satellites visible."""
        self.satellites_visible = msg.data

    def odom_callback(self, msg: Odometry):
        """Callback for local position odometry data."""
        with self.data_lock:
            self.latest_odom = msg

    def gps_raw_callback(self, msg: NavSatFix):
        """Callback for raw GPS fix data."""
        # Set reference position from first valid GPS fix
        if not self.reference_set:
            self.set_reference_position(msg)
        
        with self.data_lock:
            self.latest_gps_raw = msg

    def gps_estimated_callback(self, msg: NavSatFix):
        """Callback for estimated GPS position."""
        # Can also set reference from estimated GPS if raw not available
        if not self.reference_set:
            self.set_reference_position(msg)
            
        with self.data_lock:
            self.latest_gps_estimated = msg

    def extract_covariance_3x3(self, covariance_array, start_idx=0):
        """
        Extract 3x3 covariance matrix from ROS covariance array.
        
        Args:
            covariance_array: ROS covariance array (36 elements for 6x6)
            start_idx: Starting index (0 for position, 3 for velocity in Twist)
            
        Returns:
            List of 9 elements representing 3x3 matrix
        """
        if len(covariance_array) == 36:  # 6x6 matrix
            # Extract relevant 3x3 block
            matrix = []
            for i in range(3):
                for j in range(3):
                    idx = (start_idx + i) * 6 + (start_idx + j)
                    matrix.append(float(covariance_array[idx]))
            return matrix
        elif len(covariance_array) == 9:  # Already 3x3
            return [float(x) for x in covariance_array]
        else:
            # Return identity if unknown format
            return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

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
        
        Returns fix type: 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GPS+DR, 5=RTK Float, 6=RTK Fixed
        """
        if status < 0:
            return 0  # No fix
        elif status == 0:
            return 3  # 3D fix
        elif status == 1:
            return 4  # SBAS (augmented)
        elif status == 2:
            return 6  # GBAS (ground-based augmentation, treat as RTK)
        else:
            return 3  # Default to 3D

    def format_odom_data(self, msg: Odometry):
        """Format odometry data for DeepGIS API (compliant format)."""
        # Convert quaternion to heading
        heading = self.quaternion_to_heading(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # Extract covariances
        position_cov = self.extract_covariance_3x3(msg.pose.covariance, 0)
        velocity_cov = self.extract_covariance_3x3(msg.twist.covariance, 0)
        
        # Convert ROS timestamp to microseconds
        timestamp_usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        
        return {
            'session_id': self.session_id,
            'timestamp': datetime.now().isoformat(),
            'timestamp_usec': timestamp_usec,
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'z': float(msg.pose.pose.position.z),
            'vx': float(msg.twist.twist.linear.x),
            'vy': float(msg.twist.twist.linear.y),
            'vz': float(msg.twist.twist.linear.z),
            'heading': float(heading),
            'heading_rate': float(msg.twist.twist.angular.z),
            'position_covariance': position_cov,
            'velocity_covariance': velocity_cov,
            'ref_lat': self.ref_lat if self.ref_lat else 0.0,
            'ref_lon': self.ref_lon if self.ref_lon else 0.0,
            'ref_alt': self.ref_alt if self.ref_alt else 0.0
        }

    def format_gps_data(self, msg: NavSatFix):
        """Format GPS fix data for DeepGIS API (compliant format)."""
        # Extract accuracy
        eph, epv = self.extract_gps_accuracy(msg)
        
        # Map fix type
        fix_type = self.map_gps_fix_type(msg.status.status)
        
        # Convert ROS timestamp to microseconds
        timestamp_usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
        
        return {
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

    def publish_telemetry(self):
        """Publish telemetry data to DeepGIS API."""
        if not self.session_active:
            return

        with self.data_lock:
            odom = self.latest_odom
            gps_raw = self.latest_gps_raw
            gps_estimated = self.latest_gps_estimated

        if self.enable_batch:
            # Batch mode: accumulate data and send in batches
            with self.buffer_lock:
                if odom:
                    self.local_position_buffer.append(self.format_odom_data(odom))
                if gps_raw:
                    self.gps_raw_buffer.append(self.format_gps_data(gps_raw))
                if gps_estimated:
                    self.gps_estimated_buffer.append(self.format_gps_data(gps_estimated))

                # Check if we should send a batch
                total_samples = (len(self.local_position_buffer) +
                               len(self.gps_raw_buffer) +
                               len(self.gps_estimated_buffer))

                if total_samples >= self.batch_size:
                    self.send_batch()
        else:
            # Real-time mode: send data immediately
            if odom:
                self.send_local_position(self.format_odom_data(odom))
            if gps_raw:
                self.send_gps_raw(self.format_gps_data(gps_raw))
            if gps_estimated:
                self.send_gps_estimated(self.format_gps_data(gps_estimated))

    def send_local_position(self, data):
        """Send local position data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['local_position'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.get_logger().warn(
                    f'Failed to send local position: {response.status_code}'
                )
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending local position: {str(e)}')

    def send_gps_raw(self, data):
        """Send raw GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_raw'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.get_logger().warn(
                    f'Failed to send GPS raw: {response.status_code}'
                )
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS raw: {str(e)}')

    def send_gps_estimated(self, data):
        """Send estimated GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_estimated'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.get_logger().warn(
                    f'Failed to send GPS estimated: {response.status_code}'
                )
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS estimated: {str(e)}')

    def send_batch(self):
        """Send accumulated data in batch (API compliant format)."""
        with self.buffer_lock:
            if not (self.local_position_buffer or self.gps_raw_buffer or self.gps_estimated_buffer):
                return

            # Use API-compliant field names
            batch_data = {
                'local_position_odom': self.local_position_buffer.copy(),
                'gps_fix_raw': self.gps_raw_buffer.copy(),
                'gps_fix_estimated': self.gps_estimated_buffer.copy()
            }

            count_odom = len(self.local_position_buffer)
            count_raw = len(self.gps_raw_buffer)
            count_est = len(self.gps_estimated_buffer)

            # Clear buffers
            self.local_position_buffer.clear()
            self.gps_raw_buffer.clear()
            self.gps_estimated_buffer.clear()

        try:
            response = self.http_session.post(
                self.api_endpoints['batch'],
                json=batch_data,
                timeout=10.0
            )
            if response.status_code in [200, 201]:
                total_items = count_odom + count_raw + count_est
                self.get_logger().info(
                    f'Sent batch: {count_odom} odom, {count_raw} GPS raw, '
                    f'{count_est} GPS est ({total_items} total)'
                )
            else:
                self.get_logger().warn(
                    f'Failed to send batch: {response.status_code} - {response.text}'
                )
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error sending batch: {str(e)}')

    def destroy_node(self):
        """Cleanup before node shutdown."""
        # Send any remaining buffered data
        if self.enable_batch:
            self.send_batch()
        
        # Close HTTP session
        self.http_session.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = DeepGISTelemetryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
