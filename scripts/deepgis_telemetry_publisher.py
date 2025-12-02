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
        self.declare_parameter('api_key', 'test-missions')
        self.declare_parameter('asset_name', 'EarthRover')
        self.declare_parameter('session_id', '')  # Auto-generated if empty
        self.declare_parameter('project_title', 'EarthRover: Affordable and Sustainable Mobility Autonomy for  4D Environmental Monitoring')
        self.declare_parameter('flight_mode', 'AUTO')
        self.declare_parameter('mission_type', 'Telemetry Collection')
        self.declare_parameter('notes', 'Automated telemetry collection from MAVROS')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('batch_size', 10)
        self.declare_parameter('enable_batch_mode', True)
        self.declare_parameter('batch_flush_interval', 5.0)  # Flush batches every N seconds even if not full
        
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
        self.batch_flush_interval = self.get_parameter('batch_flush_interval').value

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
        self.last_batch_flush_time = time.time()  # Track when we last flushed a batch

        # Latest data storage
        self.latest_odom = None
        self.latest_gps_raw = None
        self.latest_gps_estimated = None
        self.data_lock = Lock()
        
        # Track last formatted data to avoid redundant formatting
        self.last_formatted_odom = None
        self.last_formatted_gps_raw = None
        self.last_formatted_gps_estimated = None
        self.last_formatted_odom_hash = None
        self.last_formatted_gps_raw_hash = None
        self.last_formatted_gps_estimated_hash = None
        
        # Cache reference position values (avoid repeated checks)
        self.cached_ref_lat = 0.0
        self.cached_ref_lon = 0.0
        self.cached_ref_alt = 0.0

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
        
        # Batch flush timer (for periodic flushing even if batch not full)
        if self.enable_batch:
            self.batch_flush_timer = self.create_timer(
                self.batch_flush_interval,
                self.flush_batch_if_needed
            )
        else:
            self.batch_flush_timer = None

        self.get_logger().info('DeepGIS Telemetry Publisher initialized')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Asset Name: {self.asset_name}')
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Batch Mode: {self.enable_batch}')

    def log_error_response(self, response, endpoint_name, request_url=None):
        """
        Log detailed error response from DeepGIS server.
        
        Args:
            response: requests.Response object
            endpoint_name: Name of the endpoint for logging context
            request_url: Optional URL that was requested
        """
        url = request_url or (response.url if hasattr(response, 'url') else 'unknown')
        
        error_msg = f'[{endpoint_name}] Request failed'
        error_msg += f'\n  URL: {url}'
        error_msg += f'\n  Status Code: {response.status_code}'
        error_msg += f'\n  Status Text: {response.reason if hasattr(response, "reason") else "N/A"}'
        
        # Try to parse JSON error response
        try:
            error_json = response.json()
            error_msg += f'\n  Response JSON: {json.dumps(error_json, indent=2)}'
        except (ValueError, json.JSONDecodeError):
            # Not JSON, show raw text
            response_text = response.text[:500]  # Limit to 500 chars
            error_msg += f'\n  Response Text: {response_text}'
            if len(response.text) > 500:
                error_msg += '... (truncated)'
        
        # Log response headers if available
        if hasattr(response, 'headers') and response.headers:
            error_msg += f'\n  Response Headers: {dict(response.headers)}'
        
        self.get_logger().error(error_msg)

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
                self.log_error_response(response, 'create_session', self.api_endpoints['create_session'])

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error creating telemetry session: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'create_session', self.api_endpoints['create_session'])
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
                # Cache the values to avoid repeated checks
                self.cached_ref_lat = self.ref_lat
                self.cached_ref_lon = self.ref_lon
                self.cached_ref_alt = self.ref_alt
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
            # Extract relevant 3x3 block (optimized with list comprehension)
            # Pre-calculate row offsets for efficiency
            row_offsets = [(start_idx + i) * 6 for i in range(3)]
            matrix = [
                float(covariance_array[row_offsets[i] + (start_idx + j)])
                for i in range(3) for j in range(3)
            ]
            return matrix
        elif len(covariance_array) == 9:  # Already 3x3
            return [float(x) for x in covariance_array]
        else:
            # Return identity if unknown format (cached constant)
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

    def _compute_odom_hash(self, msg: Odometry):
        """Compute a simple hash to detect if odom data changed."""
        # Use timestamp and position as a simple change detector
        return hash((
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ))
    
    def format_odom_data(self, msg: Odometry, use_cache=True):
        """Format odometry data for DeepGIS API (compliant format)."""
        # Check if data changed (simple optimization)
        if use_cache:
            data_hash = self._compute_odom_hash(msg)
            if data_hash == self.last_formatted_odom_hash and self.last_formatted_odom is not None:
                # Data unchanged, return cached formatted data with updated timestamp
                result = self.last_formatted_odom.copy()
                result['timestamp'] = datetime.now().isoformat()
                result['timestamp_usec'] = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
                return result
        
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
        
        result = {
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
            'ref_lat': self.cached_ref_lat,
            'ref_lon': self.cached_ref_lon,
            'ref_alt': self.cached_ref_alt
        }
        
        # Cache the result
        if use_cache:
            self.last_formatted_odom = result
            self.last_formatted_odom_hash = data_hash
        
        return result

    def _compute_gps_hash(self, msg: NavSatFix):
        """Compute a simple hash to detect if GPS data changed."""
        return hash((
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.latitude,
            msg.longitude,
            msg.altitude,
            msg.status.status
        ))
    
    def format_gps_data(self, msg: NavSatFix, use_cache=True, is_raw=True):
        """Format GPS fix data for DeepGIS API (compliant format)."""
        # Check if data changed
        if use_cache:
            data_hash = self._compute_gps_hash(msg)
            cache_hash = self.last_formatted_gps_raw_hash if is_raw else self.last_formatted_gps_estimated_hash
            cached_data = self.last_formatted_gps_raw if is_raw else self.last_formatted_gps_estimated
            
            if data_hash == cache_hash and cached_data is not None:
                # Data unchanged, return cached formatted data with updated timestamp
                result = cached_data.copy()
                result['timestamp'] = datetime.now().isoformat()
                result['timestamp_usec'] = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
                result['satellites_visible'] = int(self.satellites_visible)  # This can change
                return result
        
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
        
        # Cache the result
        if use_cache:
            if is_raw:
                self.last_formatted_gps_raw = result
                self.last_formatted_gps_raw_hash = data_hash
            else:
                self.last_formatted_gps_estimated = result
                self.last_formatted_gps_estimated_hash = data_hash
        
        return result

    def flush_batch_if_needed(self):
        """Periodically flush batches even if they're not full."""
        if not self.session_active:
            return
        
        # Timer fires at the right interval, so just flush if we have data
        # send_batch() checks for empty buffers and returns early if nothing to send
        self.send_batch()

    def publish_telemetry(self):
        """Publish telemetry data to DeepGIS API."""
        if not self.session_active:
            return

        # Get data references quickly (minimal lock time)
        with self.data_lock:
            odom = self.latest_odom
            gps_raw = self.latest_gps_raw
            gps_estimated = self.latest_gps_estimated

        if self.enable_batch:
            # Batch mode: accumulate data and send in batches
            # Format data outside the lock to reduce contention
            formatted_odom = self.format_odom_data(odom) if odom else None
            formatted_gps_raw = self.format_gps_data(gps_raw, is_raw=True) if gps_raw else None
            formatted_gps_estimated = self.format_gps_data(gps_estimated, is_raw=False) if gps_estimated else None
            
            # Now add to buffers with minimal lock time
            with self.buffer_lock:
                if formatted_odom:
                    self.local_position_buffer.append(formatted_odom)
                if formatted_gps_raw:
                    self.gps_raw_buffer.append(formatted_gps_raw)
                if formatted_gps_estimated:
                    self.gps_estimated_buffer.append(formatted_gps_estimated)

                # Check if we should send a batch immediately (when threshold reached)
                total_samples = (len(self.local_position_buffer) +
                               len(self.gps_raw_buffer) +
                               len(self.gps_estimated_buffer))

                if total_samples >= self.batch_size:
                    # send_batch() will update last_batch_flush_time
                    self.send_batch()
        else:
            # Real-time mode: send data immediately
            if odom:
                self.send_local_position(self.format_odom_data(odom, use_cache=False))
            if gps_raw:
                self.send_gps_raw(self.format_gps_data(gps_raw, use_cache=False, is_raw=True))
            if gps_estimated:
                self.send_gps_estimated(self.format_gps_data(gps_estimated, use_cache=False, is_raw=False))

    def send_local_position(self, data):
        """Send local position data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['local_position'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.log_error_response(response, 'local_position', self.api_endpoints['local_position'])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending local position: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'local_position', self.api_endpoints['local_position'])

    def send_gps_raw(self, data):
        """Send raw GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_raw'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.log_error_response(response, 'gps_raw', self.api_endpoints['gps_raw'])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS raw: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'gps_raw', self.api_endpoints['gps_raw'])

    def send_gps_estimated(self, data):
        """Send estimated GPS data to API."""
        try:
            response = self.http_session.post(
                self.api_endpoints['gps_estimated'],
                json=data,
                timeout=5.0
            )
            if response.status_code not in [200, 201]:
                self.log_error_response(response, 'gps_estimated', self.api_endpoints['gps_estimated'])
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Error sending GPS estimated: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'gps_estimated', self.api_endpoints['gps_estimated'])

    def send_batch(self):
        """Send accumulated data in batch (API compliant format)."""
        # Quick check and data extraction with minimal lock time
        with self.buffer_lock:
            if not (self.local_position_buffer or self.gps_raw_buffer or self.gps_estimated_buffer):
                return

            # Move data out of buffers (more efficient than copy for large lists)
            batch_data = {
                'local_position_odom': self.local_position_buffer,
                'gps_fix_raw': self.gps_raw_buffer,
                'gps_fix_estimated': self.gps_estimated_buffer
            }

            count_odom = len(self.local_position_buffer)
            count_raw = len(self.gps_raw_buffer)
            count_est = len(self.gps_estimated_buffer)

            # Clear buffers (create new lists instead of clearing to avoid reference issues)
            self.local_position_buffer = []
            self.gps_raw_buffer = []
            self.gps_estimated_buffer = []
            
            # Update flush time
            self.last_batch_flush_time = time.time()

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
                self.log_error_response(response, 'batch', self.api_endpoints['batch'])
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error sending batch: {str(e)}')
            if hasattr(e, 'response') and e.response is not None:
                self.log_error_response(e.response, 'batch', self.api_endpoints['batch'])

    def destroy_node(self):
        """Cleanup before node shutdown."""
        # Send any remaining buffered data
        if self.enable_batch:
            self.send_batch()
        
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
