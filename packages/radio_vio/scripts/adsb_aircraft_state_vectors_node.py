#!/usr/bin/env python3
"""
ADS-B Aircraft State Vectors Node

Subscribes to MAVROS (trike state in local/UTM frame) and ADS-B aircraft list,
and publishes all aircraft as a list of state vectors in the same local ENU
frame as the trike (origin = trike position). Each state vector includes
position, orientation, linear velocity, and angular velocity.

Uses ADS-B creatively:
  - Position: lat/lon/alt → local ENU relative to trike (same frame as MAVROS local).
  - Orientation: yaw from heading; pitch from vertical_rate and ground speed
    (climb/descent); roll assumed 0 (unknown from ADS-B).
  - Linear velocity: north/east/up from ground speed, heading, and vertical_rate
    (ft/min → m/s, positive up = climb).
  - Angular velocity: omega_z estimated from heading rate (previous sample);
    omega_x, omega_y set to 0.

Topics:
  Subscribes:
    /mavros/local_position/pose       - trike pose in local frame
    /mavros/local_position/velocity_local - trike linear velocity
    /mavros/imu/data                 - trike angular velocity (optional)
    /mavros/global_position/global  - trike LLA for origin
    /adsb/rtl_adsb_decoder_node/aircraft_list - JSON aircraft list (flight + radio)

  Publishes:
    ~/aircraft_state_vectors (std_msgs/String): JSON list of state vectors in
      frame_id (local_enu, origin at trike). Schema:
      { "header": { "frame_id", "stamp" },
        "trike": { "pose": { "position", "orientation" }, "twist": { "linear", "angular" } },
        "aircraft": [ { "icao", "pose", "twist", "raw": { "speed_kts", "heading_deg", ... } }, ... ] }
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import NavSatFix, Imu


# WGS84 approximate
EARTH_R_M = 6371000.0


def lla_to_enu_m(lat_deg: float, lon_deg: float, alt_m: float,
                 lat0_deg: float, lon0_deg: float, alt0_m: float) -> tuple:
    """Convert lat/lon/alt (deg, deg, m) to local ENU (east, north, up) in meters.
    Origin is (lat0_deg, lon0_deg, alt0_m).
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    cos_lat0 = math.cos(lat0)
    dlat = lat - lat0
    dlon = lon - lon0
    east = dlon * EARTH_R_M * cos_lat0
    north = dlat * EARTH_R_M
    up = alt_m - alt0_m
    return (east, north, up)


def quat_from_yaw_pitch(yaw_rad: float, pitch_rad: float) -> tuple:
    """Quaternion (x, y, z, w) from yaw (about up) and pitch (about east after yaw).
    Roll = 0. ENU: yaw 0 = North, 90 deg = East.
    """
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    # q_yaw = (0, 0, sy, cy), q_pitch = (0, sp, 0, cp)
    # q = q_yaw * q_pitch
    qx = cy * sp
    qy = sy * sp
    qz = sy * cp
    qw = cy * cp
    return (qx, qy, qz, qw)


def pose_to_dict(pose: Pose) -> dict:
    return {
        'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
        'orientation': {
            'x': pose.orientation.x, 'y': pose.orientation.y,
            'z': pose.orientation.z, 'w': pose.orientation.w,
        },
    }


def twist_to_dict(twist: TwistStamped) -> dict:
    t = twist.twist
    return {
        'linear': {'x': t.linear.x, 'y': t.linear.y, 'z': t.linear.z},
        'angular': {'x': t.angular.x, 'y': t.angular.y, 'z': t.angular.z},
    }


class ADSBAircraftStateVectorsNode(Node):
    def __init__(self):
        super().__init__('adsb_aircraft_state_vectors_node')

        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('aircraft_list_topic', '/adsb/rtl_adsb_decoder_node/aircraft_list')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('heading_rate_smoothing', 0.3)  # for omega_z estimate
        self.declare_parameter('mavros_local_frame', 'ned')  # 'ned' or 'enu'; output is always ENU

        self.mavros_ns = self.get_parameter('mavros_namespace').value
        self.aircraft_list_topic = self.get_parameter('aircraft_list_topic').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.heading_smoothing = float(self.get_parameter('heading_rate_smoothing').value)
        self.mavros_frame = self.get_parameter('mavros_local_frame').value

        self._lock = threading.Lock()
        self._trike_pose = None      # PoseStamped (local)
        self._trike_velocity = None  # TwistStamped (local)
        self._trike_imu = None       # Imu (angular velocity)
        self._trike_lla = None       # (lat_deg, lon_deg, alt_m) for origin
        self._aircraft_list = None   # parsed JSON dict with 'aircraft' list
        self._aircraft_list_time = 0.0

        # Per-ICAO previous heading and time for angular rate estimate
        self._prev_heading = {}
        self._prev_heading_time = {}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pose_sub = self.create_subscription(
            PoseStamped,
            f'{self.mavros_ns}/local_position/pose',
            self._pose_cb,
            qos,
        )
        self._vel_sub = self.create_subscription(
            TwistStamped,
            f'{self.mavros_ns}/local_position/velocity_local',
            self._velocity_cb,
            qos,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            f'{self.mavros_ns}/imu/data',
            self._imu_cb,
            qos,
        )
        self._global_sub = self.create_subscription(
            NavSatFix,
            f'{self.mavros_ns}/global_position/global',
            self._global_cb,
            qos,
        )
        self._aircraft_sub = self.create_subscription(
            String,
            self.aircraft_list_topic,
            self._aircraft_list_cb,
            10,
        )

        self._pub = self.create_publisher(String, '~/aircraft_state_vectors', 10)
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_cb)

        self.get_logger().info(
            'ADS-B Aircraft State Vectors node started: MAVROS=%s, aircraft_list=%s' % (
                self.mavros_ns, self.aircraft_list_topic)
        )

    def _pose_cb(self, msg: PoseStamped):
        with self._lock:
            self._trike_pose = msg

    def _velocity_cb(self, msg: TwistStamped):
        with self._lock:
            self._trike_velocity = msg

    def _imu_cb(self, msg: Imu):
        with self._lock:
            self._trike_imu = msg

    def _global_cb(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        with self._lock:
            # NavSatFix.altitude is typically meters (AMSL or ellipsoid)
            self._trike_lla = (msg.latitude, msg.longitude, float(msg.altitude))

    def _aircraft_list_cb(self, msg: String):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(data.get('aircraft'), list):
            return
        with self._lock:
            self._aircraft_list = data
            self._aircraft_list_time = time.time()

    def _publish_cb(self):
        with self._lock:
            trike_pose = self._trike_pose
            trike_velocity = self._trike_velocity
            trike_imu = self._trike_imu
            trike_lla = self._trike_lla
            aircraft_list = self._aircraft_list

        if trike_lla is None:
            return
        if aircraft_list is None or not aircraft_list.get('aircraft'):
            return

        now = time.time()
        frame_id = 'local_enu'
        stamp_sec = int(now)
        stamp_nanosec = int((now % 1.0) * 1e9)

        # Trike state: convert to ENU if MAVROS gives NED (x=N, y=E, z=-D) so output matches aircraft (ENU)
        trike_json = {}
        if trike_pose:
            p = trike_pose.pose.position
            o = trike_pose.pose.orientation
            if self.mavros_frame.lower() == 'ned':
                trike_json['pose'] = {
                    'position': {'x': p.y, 'y': p.x, 'z': -p.z},
                    'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w},
                }
            else:
                trike_json['pose'] = pose_to_dict(trike_pose.pose)
        else:
            trike_json['pose'] = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
        if trike_velocity:
            t = trike_velocity.twist
            if self.mavros_frame.lower() == 'ned':
                trike_json['twist'] = {
                    'linear': {'x': t.linear.y, 'y': t.linear.x, 'z': -t.linear.z},
                    'angular': {'x': t.angular.x, 'y': t.angular.y, 'z': t.angular.z},
                }
            else:
                trike_json['twist'] = twist_to_dict(trike_velocity)
        else:
            trike_json['twist'] = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        if trike_imu and trike_imu.angular_velocity:
            trike_json['twist']['angular'] = {
                'x': trike_imu.angular_velocity.x,
                'y': trike_imu.angular_velocity.y,
                'z': trike_imu.angular_velocity.z,
            }

        lat0, lon0, alt0_m = trike_lla
        aircraft_states = []

        for a in aircraft_list['aircraft']:
            icao = (a.get('icao') or '').strip().upper()
            if not icao or len(icao) != 6:
                continue
            lat = a.get('latitude')
            lon = a.get('longitude')
            alt_ft = a.get('altitude')
            if lat is None or lon is None:
                continue
            # Altitude: aircraft_list uses feet from ADS-B
            alt_m = (float(alt_ft) * 0.3048) if alt_ft is not None else 0.0

            east, north, up = lla_to_enu_m(lat, lon, alt_m, lat0, lon0, alt0_m)

            # Speed (knots), heading (deg), vertical_rate (ft/min, positive = climb)
            speed_kts = a.get('speed')
            heading_deg = a.get('heading')
            vertical_rate_ftmin = a.get('vertical_rate')

            speed_kts = float(speed_kts) if speed_kts is not None else 0.0
            heading_deg = float(heading_deg) if heading_deg is not None else 0.0
            vertical_rate_ftmin = float(vertical_rate_ftmin) if vertical_rate_ftmin is not None else 0.0

            heading_rad = math.radians(heading_deg)
            speed_mps = speed_kts * 0.514444
            vertical_rate_mps = vertical_rate_ftmin * (1.0 / 60.0) * 0.3048  # ft/min → m/s, positive = up

            # Orientation: yaw from heading; pitch from climb/descent vs ground speed
            pitch_rad = 0.0
            if speed_mps > 1.0:
                pitch_rad = math.atan2(vertical_rate_mps, speed_mps)
                pitch_rad = max(-0.5 * math.pi, min(0.5 * math.pi, pitch_rad))
            qx, qy, qz, qw = quat_from_yaw_pitch(heading_rad, pitch_rad)

            # Linear velocity in ENU: v_east, v_north, v_up
            v_east = speed_mps * math.sin(heading_rad)
            v_north = speed_mps * math.cos(heading_rad)
            v_up = vertical_rate_mps

            # Angular velocity: estimate omega_z from heading rate
            omega_z = 0.0
            if icao in self._prev_heading and icao in self._prev_heading_time:
                dt = now - self._prev_heading_time[icao]
                if dt > 0 and dt < 10.0:
                    d_heading = heading_rad - self._prev_heading[icao]
                    d_heading = (d_heading + math.pi) % (2.0 * math.pi) - math.pi
                    omega_z = self.heading_smoothing * (d_heading / dt) + (1.0 - self.heading_smoothing) * 0.0
            self._prev_heading[icao] = heading_rad
            self._prev_heading_time[icao] = now

            aircraft_states.append({
                'icao': icao,
                'pose': {
                    'position': {'x': east, 'y': north, 'z': up},
                    'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw},
                },
                'twist': {
                    'linear': {'x': v_east, 'y': v_north, 'z': v_up},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': omega_z},
                },
                'raw': {
                    'speed_kts': round(speed_kts, 1),
                    'heading_deg': round(heading_deg, 1),
                    'vertical_rate_ftmin': round(vertical_rate_ftmin, 0),
                    'altitude_ft': round(alt_ft, 0) if alt_ft is not None else None,
                    'latitude': round(lat, 6),
                    'longitude': round(lon, 6),
                },
            })

        payload = {
            'header': {
                'frame_id': frame_id,
                'stamp': {'sec': stamp_sec, 'nanosec': stamp_nanosec},
                'origin_lla': {'lat': lat0, 'lon': lon0, 'alt_m': alt0_m},
            },
            'trike': trike_json,
            'aircraft': aircraft_states,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ADSBAircraftStateVectorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
