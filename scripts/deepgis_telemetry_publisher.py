#!/usr/bin/env python3
"""
DeepGIS Telemetry Publisher Node

Subscribes to MAVROS telemetry topics and logs them to the DeepGIS telemetry API.

Every sample is written to a local SQLite spool first; an uploader thread sends the
oldest samples as gzip batches, at most upload_kbps, and deletes them only once the
server has stored them. A link outage (Starlink standby, no signal, a server restart)
therefore delays samples instead of losing them. The server keeps one record per
(session, timestamp_usec), so a batch resent after an unclear failure is harmless.

One session per boot: the session id is kept with the boot id in the state directory,
so restarting the node continues the same session. While the vehicle is parked (slower
than stationary_speed and within stationary_radius_m of the last logged position) it is
logged every stationary_interval seconds instead of at publish_rate.
"""

import gzip
import os
import sqlite3
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt32

import requests
import json
import math
from datetime import datetime, timezone
from threading import Lock
import time

KINDS = ('local_position_odom', 'gps_fix_raw', 'gps_fix_estimated')
BACKOFF_MAX_SEC = 120.0


class TelemetrySpool:
    """Samples waiting for upload, in SQLite (WAL); one connection per thread."""

    def __init__(self, path, max_rows):
        self.path = path
        self.max_rows = max_rows
        self.local = threading.local()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        db = self.db()
        db.execute('PRAGMA journal_mode=WAL')
        db.execute('CREATE TABLE IF NOT EXISTS samples (id INTEGER PRIMARY KEY, kind TEXT NOT NULL, '
                   'session_id TEXT NOT NULL, ts_usec INTEGER NOT NULL, payload TEXT NOT NULL, '
                   'UNIQUE (kind, session_id, ts_usec))')
        db.commit()

    def db(self):
        if not hasattr(self.local, 'db'):
            self.local.db = sqlite3.connect(self.path, timeout=10)
        return self.local.db

    def add(self, rows):
        """rows: [(kind, payload dict)]; a sample already spooled is ignored."""
        db = self.db()
        db.executemany('INSERT OR IGNORE INTO samples (kind, session_id, ts_usec, payload) VALUES (?, ?, ?, ?)',
                       [(k, p['session_id'], p['timestamp_usec'], json.dumps(p, separators=(',', ':')))
                        for k, p in rows])
        db.commit()

    def oldest(self, limit):
        return self.db().execute('SELECT id, kind, payload FROM samples ORDER BY id LIMIT ?', (limit,)).fetchall()

    def delete(self, ids):
        db = self.db()
        db.executemany('DELETE FROM samples WHERE id = ?', [(i,) for i in ids])
        db.commit()

    def count(self):
        return self.db().execute('SELECT COUNT(*) FROM samples').fetchone()[0]

    def trim(self):
        """Drop the oldest samples beyond max_rows (weeks offline); returns how many."""
        extra = self.count() - self.max_rows
        if extra <= 0:
            return 0
        db = self.db()
        db.execute('DELETE FROM samples WHERE id IN (SELECT id FROM samples ORDER BY id LIMIT ?)', (extra,))
        db.commit()
        return extra


def stamp_fields(msg):
    """timestamp (UTC ISO 8601) and timestamp_usec from the message header, or now if unstamped."""
    usec = msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec // 1000
    if usec <= 0:
        usec = time.time_ns() // 1000
    return {'timestamp': datetime.fromtimestamp(usec / 1e6, timezone.utc).isoformat(),
            'timestamp_usec': usec}


def boot_session_id(state_dir, prefix):
    """The session id for this boot, created on first use and reused after restarts."""
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            boot_id = f.read().strip()
    except OSError:
        boot_id = ''
    path = os.path.join(state_dir, 'telemetry_session.json')
    try:
        with open(path) as f:
            saved = json.load(f)
        if boot_id and saved.get('boot_id') == boot_id and saved.get('session_id'):
            return saved['session_id']
    except (OSError, ValueError):
        pass
    session_id = f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    os.makedirs(state_dir, exist_ok=True)
    with open(path, 'w') as f:
        json.dump({'boot_id': boot_id, 'session_id': session_id}, f)
    return session_id


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
        state_dir = os.path.expanduser('~/.local/share/earth-rover')
        self.declare_parameter('spool_path', os.path.join(state_dir, 'telemetry_spool.sqlite'))
        self.declare_parameter('spool_max_rows', 2000000)  # ~11 days of 1 Hz odom + GPS offline
        self.declare_parameter('upload_interval', 5.0)  # seconds between batches once caught up
        self.declare_parameter('upload_kbps', 50.0)  # cap on compressed upload rate
        self.declare_parameter('max_batch_rows', 600)
        self.declare_parameter('stationary_interval', 10.0)
        self.declare_parameter('stationary_speed', 0.2)
        self.declare_parameter('stationary_radius_m', 2.0)
        
        # Get parameters
        self.api_url = self.get_parameter('deepgis_api_url').value
        self.api_key = self.get_parameter('api_key').value
        self.asset_name = self.get_parameter('asset_name').value
        session_id_param = self.get_parameter('session_id').value
        self.session_id = session_id_param or boot_session_id(state_dir, self.asset_name)
        self.project_title = self.get_parameter('project_title').value
        self.flight_mode = self.get_parameter('flight_mode').value
        self.mission_type = self.get_parameter('mission_type').value
        self.notes = self.get_parameter('notes').value
        self.mavros_ns = self.get_parameter('mavros_namespace').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.upload_interval = float(self.get_parameter('upload_interval').value)
        self.upload_bps = max(1.0, float(self.get_parameter('upload_kbps').value)) * 1000.0
        self.max_batch_rows = int(self.get_parameter('max_batch_rows').value)
        self.stationary_interval = float(self.get_parameter('stationary_interval').value)
        self.stationary_speed = float(self.get_parameter('stationary_speed').value)
        self.stationary_radius_m = float(self.get_parameter('stationary_radius_m').value)
        self.spool = TelemetrySpool(os.path.expanduser(self.get_parameter('spool_path').value),
                                    int(self.get_parameter('spool_max_rows').value))

        # API endpoints
        self.api_endpoints = {
            'create_session': f'{self.api_url}/api/telemetry/session/create/',
            'local_position': f'{self.api_url}/api/telemetry/local-position-odom/',
            'gps_raw': f'{self.api_url}/api/telemetry/gps-fix-raw/',
            'gps_estimated': f'{self.api_url}/api/telemetry/gps-fix-estimated/',
            'batch': f'{self.api_url}/api/telemetry/batch/',
        }

        # Sessions the server has confirmed (the uploader creates them before their first batch)
        self.sessions_created = set()

        # Reference position (set from first valid GPS fix)
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.reference_set = False
        
        # Satellite count
        self.satellites_visible = 0

        # Parked-vehicle deadband: where and when the last sample was logged
        self.last_logged_t = 0.0
        self.last_logged_fix = None

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

        # Sampling timer (ROS executor) and uploader thread (network)
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_telemetry
        )
        self.stop_upload = threading.Event()
        self.uploader = threading.Thread(target=self.upload_loop, name='deepgis-uploader', daemon=True)
        self.uploader.start()

        self.get_logger().info('DeepGIS Telemetry Publisher initialized')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Asset Name: {self.asset_name}')
        self.get_logger().info(f'Session ID: {self.session_id}')
        self.get_logger().info(f'Spool: {self.spool.path} ({self.spool.count()} samples waiting), '
                               f'upload cap {self.upload_bps / 1000:.0f} kbit/s')

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

    def create_telemetry_session(self, session_id):
        """Create (or confirm) a telemetry session on the server; True once it exists there."""
        try:
            payload = {
                'session_id': session_id,
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
                verb = 'Created' if response.status_code == 201 else 'Continuing'
                self.get_logger().info(f'{verb} telemetry session: {session_id}')
                return True
            self.log_error_response(response, 'create_session', self.api_endpoints['create_session'])

        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Session {session_id} not created yet: {type(e).__name__}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error creating session: {str(e)}')
        return False

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
                result.update(stamp_fields(msg))
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
        
        
        result = {
            'session_id': self.session_id,
            **stamp_fields(msg),
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
                result.update(stamp_fields(msg))
                result['satellites_visible'] = int(self.satellites_visible)  # This can change
                return result
        
        # Extract accuracy
        eph, epv = self.extract_gps_accuracy(msg)
        
        # Map fix type
        fix_type = self.map_gps_fix_type(msg.status.status)
        
        
        result = {
            'session_id': self.session_id,
            **stamp_fields(msg),
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

    def _parked(self, odom, fix, now):
        """True while the vehicle is slow and near the last logged fix, and that was recent."""
        if self.last_logged_fix is None or now - self.last_logged_t >= self.stationary_interval:
            return False
        if odom is not None:
            v = odom.twist.twist.linear
            if math.hypot(v.x, v.y) >= self.stationary_speed:
                return False
        if fix is None:
            return odom is not None
        lat0, lon0 = self.last_logged_fix
        dn = math.radians(fix.latitude - lat0) * 6371000.0
        de = math.radians(fix.longitude - lon0) * 6371000.0 * math.cos(math.radians(lat0))
        return math.hypot(dn, de) < self.stationary_radius_m

    def publish_telemetry(self):
        """Spool the latest samples (at publish_rate, or every stationary_interval when parked)."""
        with self.data_lock:
            odom = self.latest_odom
            gps_raw = self.latest_gps_raw
            gps_estimated = self.latest_gps_estimated
        fix = gps_raw if gps_raw is not None and gps_raw.status.status >= 0 else None
        now = time.monotonic()
        if self._parked(odom, fix, now):
            return

        rows = []
        if odom:
            rows.append(('local_position_odom', self.format_odom_data(odom)))
        if gps_raw:
            rows.append(('gps_fix_raw', self.format_gps_data(gps_raw, is_raw=True)))
        if gps_estimated:
            rows.append(('gps_fix_estimated', self.format_gps_data(gps_estimated, is_raw=False)))
        if not rows:
            return
        try:
            self.spool.add(rows)
        except sqlite3.Error as e:
            self.get_logger().error(f'Could not spool telemetry: {e}')
            return
        self.last_logged_t = now
        if fix is not None:
            self.last_logged_fix = (fix.latitude, fix.longitude)

    def upload_loop(self):
        """Drain the spool oldest-first: gzip batches, at most upload_bps, backing off when the link fails."""
        limit = self.max_batch_rows
        backoff = 0.0
        next_trim = 0.0
        while not self.stop_upload.is_set():
            if time.monotonic() >= next_trim:
                next_trim = time.monotonic() + 60.0
                try:
                    dropped = self.spool.trim()
                    if dropped:
                        self.get_logger().warn(f'Spool full: dropped the {dropped} oldest samples')
                except sqlite3.Error as e:
                    self.get_logger().error(f'Spool trim failed: {e}')
            try:
                outcome, sent_bytes, rows = self.upload_once(limit)
            except Exception as e:
                self.get_logger().error(f'Uploader error: {e}')
                outcome, sent_bytes, rows = 'retry', 0, 0

            if outcome == 'retry':
                backoff = min(BACKOFF_MAX_SEC, max(5.0, backoff * 2))
                self.stop_upload.wait(backoff)
                continue
            backoff = 0.0
            if outcome == 'shrink':
                limit = max(1, rows // 2)
                continue
            pause = sent_bytes * 8.0 / self.upload_bps
            if outcome == 'sent':
                if rows < limit:
                    pause = max(pause, self.upload_interval)
                limit = min(self.max_batch_rows, limit * 2)
            else:
                pause = self.upload_interval
            self.stop_upload.wait(pause)

    def upload_once(self, limit):
        """Send one batch; returns (outcome, compressed bytes sent, rows in the batch)."""
        rows = self.spool.oldest(limit)
        if not rows:
            return 'idle', 0, 0
        batch = {kind: [] for kind in KINDS}
        sessions = set()
        for _id, kind, payload in rows:
            record = json.loads(payload)
            batch[kind].append(record)
            sessions.add(record['session_id'])
        for session_id in sorted(sessions - self.sessions_created):
            if not self.create_telemetry_session(session_id):
                return 'retry', 0, len(rows)
            self.sessions_created.add(session_id)

        body = gzip.compress(json.dumps(batch, separators=(',', ':')).encode())
        url = self.api_endpoints['batch']
        try:
            response = self.http_session.post(url, data=body, timeout=30.0,
                                              headers={'Content-Encoding': 'gzip'})
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'Batch upload failed ({type(e).__name__}); '
                                   f'{self.spool.count()} samples waiting')
            return 'retry', 0, len(rows)

        ids = [r[0] for r in rows]
        if response.status_code in (201, 207):
            self.spool.delete(ids)
            try:
                results = response.json().get('results', {})
            except ValueError:
                results = {}
            created = sum(r.get('created', 0) for r in results.values())
            dups = sum(r.get('duplicates', 0) for r in results.values())
            errors = sum(len(r.get('errors', [])) for r in results.values())
            msg = f'Uploaded {len(rows)} samples in {len(body)} B: {created} new, {dups} already stored'
            if errors:
                self.get_logger().warn(f'{msg}, {errors} rejected; first: '
                                       f'{next(e for r in results.values() for e in r.get("errors", []))}')
            else:
                self.get_logger().info(msg)
            return 'sent', len(body), len(rows)
        if response.status_code in (400, 413):
            if len(rows) > 1:
                return 'shrink', 0, len(rows)
            self.get_logger().error(f'Server rejects one sample outright, dropping it: {rows[0][2][:200]}')
            self.spool.delete(ids)
            return 'sent', len(body), 1
        self.log_error_response(response, 'batch', url)
        return 'retry', 0, len(rows)

    def destroy_node(self):
        """Cleanup before node shutdown; unsent samples stay in the spool for the next run."""
        self.stop_upload.set()
        self.uploader.join(timeout=10.0)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
