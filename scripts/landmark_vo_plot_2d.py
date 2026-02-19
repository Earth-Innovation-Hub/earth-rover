#!/usr/bin/env python3
"""
2D Landmark VO Plot Node

Subscribes to odom, estimated_position, landmarks, observations, camera_info,
and aircraft_state_vectors. Renders:
  - Left panel: Image plane – predicted (green circle) vs actual (red x) landmarks
    with epipolar residuals (cyan arrows); aircraft projections (orange).
  - Right panel: World 2D view – HFoV cone, vehicle marker, landmarks, aircraft.

Uses PinholeCameraModel for Grasshopper/Chameleon camera. Body frame: X forward,
Y left, Z up. ROS optical: X right, Y down, Z forward.

Ref bootstrap from global_position and raw/fix. Estimated position parsing
(kf_lat/kf_lon or lat/lon) same as VCS.

Usage:
  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py
  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py demo_mode:=true
"""

import json
import math
import sys
import threading

def _import_matplotlib():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        return plt
    except (ImportError, AttributeError) as e:
        err = str(e)
        if 'numpy' in err.lower() or 'ARRAY_API' in err or 'multiarray' in err:
            print('Matplotlib/NumPy version conflict. Fix: pip install "numpy<2"\n', file=sys.stderr)
        raise

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, SetParametersResult
from matplotlib.lines import Line2D
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from std_msgs.msg import Header

EARTH_R_M = 6371000.0
M_PER_KM = 1000.0

# Colorblind-friendly palette: blue=actual, orange=KF/predicted
COLOR_ACTUAL = '#2196F3'       # Material blue
COLOR_ACTUAL_EDGE = '#1565C0'  # Darker blue
COLOR_KF = '#FF9800'           # Material orange
COLOR_KF_EDGE = '#FFB74D'      # Lighter orange
COLOR_INNOVATION = '#00BCD4'    # Cyan
COLOR_LANDMARKS = '#9C27B0'     # Purple (distinct from actual blue)
COLOR_AIRCRAFT = '#26A69A'     # Teal (distinct from KF orange)


def lla_to_enu_m(lat_deg: float, lon_deg: float, alt_m: float,
                 lat0_deg: float, lon0_deg: float, alt0_m: float) -> tuple:
    """Convert lat/lon/alt (deg, deg, m) to local ENU (east, north, up) in meters."""
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


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (radians) from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PinholeCameraModel:
    """Pinhole camera model for Grasshopper/Chameleon. ROS optical frame."""

    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 width: int = 1280, height: int = 1024):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height

    def project(self, x_cam: float, y_cam: float, z_cam: float):
        """Project 3D point in camera frame to 2D (u, v). Returns (u, v) or None if behind."""
        if z_cam <= 1e-6:
            return None
        u = self.fx * (x_cam / z_cam) + self.cx
        v = self.fy * (y_cam / z_cam) + self.cy
        if u < 0 or u >= self.width or v < 0 or v >= self.height:
            return (u, v)  # Allow outside frame for visualization
        return (u, v)

    def hfov_rad(self) -> float:
        """Horizontal FOV in radians."""
        return 2.0 * math.atan(self.width / (2.0 * self.fx))

    def vfov_rad(self) -> float:
        """Vertical FOV in radians."""
        return 2.0 * math.atan(self.height / (2.0 * self.fy))


def world_to_camera(x_w: float, y_w: float, z_w: float,
                    cam_x: float, cam_y: float, cam_z: float,
                    yaw_rad: float, heading_offset_rad: float = 0.0,
                    pitch_offset_rad: float = 0.0):
    """
    Transform world (ENU) point to camera frame.
    Body: X forward, Y left, Z up. ROS optical: X right, Y down, Z forward.
    body X → cam Z, body Y → cam −X, body Z → cam −Y.
    heading_offset adds to yaw; pitch_offset tilts view up (positive) / down (negative).
    """
    effective_yaw = yaw_rad + heading_offset_rad
    c = math.cos(-effective_yaw)
    s = math.sin(-effective_yaw)
    dx = x_w - cam_x
    dy = y_w - cam_y
    dz = z_w - cam_z
    # World to body (yaw only)
    lx_b = c * dx - s * dy
    ly_b = s * dx + c * dy
    lz_b = dz
    # Pitch: rotate about body Y. Positive pitch = nose up.
    cp = math.cos(-pitch_offset_rad)
    sp = math.sin(-pitch_offset_rad)
    lx_bp = cp * lx_b + sp * lz_b
    lz_bp = -sp * lx_b + cp * lz_b
    # Body to camera optical: x_cam = -ly_b, y_cam = -lz_b, z_cam = lx_b
    x_cam = -ly_b
    y_cam = -lz_bp
    z_cam = lx_bp
    return (x_cam, y_cam, z_cam)


def world_to_camera_3d(x_w: float, y_w: float, z_w: float,
                       cam_x: float, cam_y: float, cam_z: float,
                       yaw_rad: float, heading_offset_rad: float = 0.0,
                       pitch_offset_rad: float = 0.0):
    """Same as world_to_camera; alias for 3D points."""
    return world_to_camera(x_w, y_w, z_w, cam_x, cam_y, cam_z,
                           yaw_rad, heading_offset_rad, pitch_offset_rad)


class LandmarkVOPlot2DNode(Node):
    def __init__(self):
        super().__init__('landmark_vo_plot_2d')

        # Parameters from config/landmark_vo_plot_2d.yaml
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('pose_topic', '')
        self.declare_parameter('estimated_position_topic', '/adsb/rtl_adsb_decoder_node/estimated_position')
        self.declare_parameter('mavros_local_frame', 'enu')
        self.declare_parameter('landmarks_topic', '/vo/landmarks')
        self.declare_parameter('observations_topic', '/vo/landmark_observations')
        self.declare_parameter('camera_info_topic', '/stereo/left/camera_info')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 800)
        self.declare_parameter('camera_fx', 1000.0)
        self.declare_parameter('camera_fy', 1000.0)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 512.0)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 1024)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_height', 1.2)
        self.declare_parameter('camera_heading_offset_deg', 0.0)
        self.declare_parameter('yaw_zero_direction', 'east')  # 'east'=ENU/ROS, 'north'=NED/aviation
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('adsb_state_vectors_topic', '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors')
        self.declare_parameter('aircraft_observations_topic', '')
        self.declare_parameter('enable_aircraft', True)
        self.declare_parameter('aircraft_max_range_km', 8.0)
        self.declare_parameter('aircraft_min_range_m', 50.0)
        self.declare_parameter('world_view_extent_km', 10.0)
        self.declare_parameter('mavros_namespace', '/mavros')

        # Dynamic trim params for rqt_reconfigure (real-time yaw/pitch offset)
        yaw_trim_desc = ParameterDescriptor(
            description='Yaw trim (deg). Add to trike yaw to emulate other views.',
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=-180.0, to_value=180.0, step=0.5)],
        )
        pitch_trim_desc = ParameterDescriptor(
            description='Pitch trim (deg). Tilt view up (+) / down (-) to emulate other views.',
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=-90.0, to_value=90.0, step=0.5)],
        )
        self.declare_parameter('yaw_trim_deg', 0.0, yaw_trim_desc)
        self.declare_parameter('pitch_trim_deg', 0.0, pitch_trim_desc)

        self._odom_topic = self.get_parameter('odom_topic').value
        self._est_pos_topic = self.get_parameter('estimated_position_topic').value
        self._landmarks_topic = self.get_parameter('landmarks_topic').value
        self._obs_topic = self.get_parameter('observations_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._adsb_topic = self.get_parameter('adsb_state_vectors_topic').value
        self._mavros_ns = self.get_parameter('mavros_namespace').value
        self._mavros_frame = self.get_parameter('mavros_local_frame').value
        self._demo_mode = self.get_parameter('demo_mode').value
        self._enable_aircraft = self.get_parameter('enable_aircraft').value
        self._aircraft_max_km = float(self.get_parameter('aircraft_max_range_km').value)
        self._aircraft_min_m = float(self.get_parameter('aircraft_min_range_m').value)
        self._world_extent_km = float(self.get_parameter('world_view_extent_km').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._cam_fx = float(self.get_parameter('camera_fx').value)
        self._cam_fy = float(self.get_parameter('camera_fy').value)
        self._cam_cx = float(self.get_parameter('camera_cx').value)
        self._cam_cy = float(self.get_parameter('camera_cy').value)
        self._cam_w = int(self.get_parameter('camera_width').value)
        self._cam_h = int(self.get_parameter('camera_height').value)
        self._cam_offset_x = float(self.get_parameter('cam_offset_x').value)
        self._cam_offset_y = float(self.get_parameter('cam_offset_y').value)
        self._cam_height = float(self.get_parameter('cam_height').value)
        self._heading_offset_deg = float(self.get_parameter('camera_heading_offset_deg').value)
        self._heading_offset_rad = math.radians(self._heading_offset_deg)
        self._yaw_trim_deg = float(self.get_parameter('yaw_trim_deg').value)
        self._pitch_trim_deg = float(self.get_parameter('pitch_trim_deg').value)
        yaw_zero = str(self.get_parameter('yaw_zero_direction').value or 'east').lower()
        self._yaw_zero_east = (yaw_zero == 'east')

        self._camera = PinholeCameraModel(
            self._cam_fx, self._cam_fy, self._cam_cx, self._cam_cy,
            self._cam_w, self._cam_h
        )

        self._lock = threading.Lock()
        self._odom = None
        self._est_pos_json = None
        self._landmarks = []
        self._observations = []
        self._camera_info_received = False
        self._aircraft_data = None
        self._ref_lat = None
        self._ref_lon = None
        self._ref_alt = 0.0
        self._ref_set = False
        self._frame_count = 0

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, qos)
        if self._est_pos_topic:
            self.create_subscription(String, self._est_pos_topic, self._est_pos_cb, 10)
        self.create_subscription(String, self._landmarks_topic, self._landmarks_cb, 10)
        self.create_subscription(String, self._obs_topic, self._observations_cb, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 10)

        self.add_on_set_parameters_callback(self._parameters_cb)
        if self._enable_aircraft and self._adsb_topic:
            self.create_subscription(String, self._adsb_topic, self._aircraft_cb, 10)

        # Ref bootstrap from global_position and raw/fix
        self.create_subscription(
            NavSatFix, f'{self._mavros_ns}/global_position/global',
            self._global_cb, qos
        )
        self.create_subscription(
            NavSatFix, f'{self._mavros_ns}/global_position/raw/fix',
            self._raw_fix_cb, qos
        )

        self._image_pub = self.create_publisher(Image, '~/landmark_vo_plot_image', 10)
        self._fig = None
        self._ax_img = None
        self._ax_world = None
        self._plt = None
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_plot)

        self.get_logger().info(
            'Landmark VO Plot 2D: odom=%s, est=%s, landmarks=%s, obs=%s' % (
                self._odom_topic, self._est_pos_topic, self._landmarks_topic, self._obs_topic)
        )

    def _set_ref(self, lat: float, lon: float, alt: float):
        if not self._ref_set and not (math.isnan(lat) or math.isnan(lon)):
            self._ref_lat = lat
            self._ref_lon = lon
            self._ref_alt = alt if not math.isnan(alt) else 0.0
            self._ref_set = True
            self.get_logger().info('Ref: lat=%.6f lon=%.6f alt=%.1f' % (lat, lon, self._ref_alt))

    def _global_cb(self, msg: NavSatFix):
        if msg.latitude != 0 or msg.longitude != 0:
            self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))

    def _raw_fix_cb(self, msg: NavSatFix):
        try:
            if msg.status.status >= 0 and not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))
        except (AttributeError, TypeError):
            if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
                self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._odom = msg

    def _est_pos_cb(self, msg: String):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._est_pos_json = data

    def _landmarks_cb(self, msg: String):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        lst = data.get('landmarks')
        if isinstance(lst, list):
            with self._lock:
                self._landmarks = [{'id': lm.get('id'), 'x': float(lm.get('x', 0)), 'y': float(lm.get('y', 0))}
                                  for lm in lst if isinstance(lm, dict)]

    def _observations_cb(self, msg: String):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        lst = data.get('observations')
        if isinstance(lst, list):
            with self._lock:
                self._observations = [{'id': o.get('id'), 'u': float(o.get('u', 0)), 'v': float(o.get('v', 0))}
                                     for o in lst if isinstance(o, dict)]

    def _camera_info_cb(self, msg: CameraInfo):
        if msg.k and len(msg.k) >= 6 and msg.k[0] != 0:
            with self._lock:
                self._cam_fx = msg.k[0]
                self._cam_fy = msg.k[4]
                self._cam_cx = msg.k[2]
                self._cam_cy = msg.k[5]
                if msg.width and msg.height:
                    self._cam_w = msg.width
                    self._cam_h = msg.height
                self._camera = PinholeCameraModel(
                    self._cam_fx, self._cam_fy, self._cam_cx, self._cam_cy,
                    self._cam_w, self._cam_h
                )
                self._camera_info_received = True

    def _aircraft_cb(self, msg: String):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._aircraft_data = data

    def _parameters_cb(self, params):
        """Handle dynamic parameter changes from rqt_reconfigure."""
        result = SetParametersResult(successful=True)
        for p in params:
            if p.name == 'yaw_trim_deg':
                self._yaw_trim_deg = float(p.value)
            elif p.name == 'pitch_trim_deg':
                self._pitch_trim_deg = float(p.value)
        return result

    def _get_pose(self):
        """Return (x, y, yaw, use_estimated) in local ENU."""
        with self._lock:
            odom = self._odom
            est = self._est_pos_json
            ref_set = self._ref_set
            ref_lat = self._ref_lat
            ref_lon = self._ref_lon
            ref_alt = self._ref_alt

        if not odom:
            return 0.0, 0.0, 0.0, False

        # Odom position/orientation (handle NED if needed)
        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        if self._mavros_frame.lower() == 'ned':
            ox, oy, oz = p.y, p.x, -p.z
        else:
            ox, oy, oz = p.x, p.y, p.z
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

        if not self._est_pos_topic or not est or not ref_set:
            return ox, oy, yaw, False

        # Estimated position parsing: kf_lat/kf_lon or lat/lon (same as VCS)
        has_kf = est.get('kf_lat') is not None and est.get('kf_lon') is not None
        has_raw = est.get('raw_lat') is not None and est.get('raw_lon') is not None
        lat = (est.get('kf_lat') if has_kf else est.get('lat')) or (est.get('raw_lat') if has_raw else est.get('lat'))
        lon = (est.get('kf_lon') if has_kf else est.get('lon')) or (est.get('raw_lon') if has_raw else est.get('lon'))

        if lat is None or lon is None:
            return ox, oy, yaw, False

        alt = float(est.get('altitude', ref_alt) or ref_alt)
        ex, ey, ez = lla_to_enu_m(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        return ex, ey, yaw, True

    def _get_kf_odom_offset(self):
        """Return (dx, dy) = KF_pos - odom_pos when using KF, else (0,0)."""
        with self._lock:
            odom = self._odom
            est = self._est_pos_json
            ref_set = self._ref_set

        if not odom or not self._est_pos_topic or not est or not ref_set:
            return 0.0, 0.0

        has_kf = est.get('kf_lat') is not None and est.get('kf_lon') is not None
        if not has_kf:
            return 0.0, 0.0

        lat = est.get('kf_lat')
        lon = est.get('kf_lon')
        if lat is None or lon is None:
            return 0.0, 0.0

        alt = float(est.get('altitude', self._ref_alt) or self._ref_alt)
        kx, ky, _ = lla_to_enu_m(lat, lon, alt, self._ref_lat, self._ref_lon, self._ref_alt)
        p = odom.pose.pose.position
        if self._mavros_frame.lower() == 'ned':
            ox, oy = p.y, p.x
        else:
            ox, oy = p.x, p.y
        return kx - ox, ky - oy

    def _setup_figure(self):
        if self._fig is not None:
            return
        self._plt = _import_matplotlib()
        plt = self._plt
        self._fig, (self._ax_img, self._ax_world) = plt.subplots(1, 2, figsize=(16, 8), dpi=100)
        self._ax_img.set_xlim(0, self._cam_w)
        self._ax_img.set_ylim(self._cam_h, 0)
        self._ax_img.set_aspect('equal')
        self._ax_img.set_xlabel('u (px)', fontsize=8)
        self._ax_img.set_ylabel('v (px)', fontsize=8)
        self._ax_img.set_title('Image plane (predicted vs actual)', fontsize=9)
        self._ax_img.grid(True, alpha=0.4)

        extent_m = self._world_extent_km * M_PER_KM * 0.5
        self._ax_world.set_xlim(-extent_m, extent_m)
        self._ax_world.set_ylim(-extent_m, extent_m)
        self._ax_world.set_aspect('equal')
        self._ax_world.set_xlabel('East (m)', fontsize=8)
        self._ax_world.set_ylabel('North (m)', fontsize=8)
        self._ax_world.set_title('World 2D (centered on vehicle)', fontsize=9)
        self._ax_world.grid(True, alpha=0.4)
        self._ax_world.axhline(0, color='k', linewidth=0.5)
        self._ax_world.axvline(0, color='k', linewidth=0.5)
        self._fig.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.08)
        self._fig.patch.set_facecolor('white')
        self._ax_img.set_facecolor('#1a1a1a')
        self._ax_world.set_facecolor('#1a1a1a')

    def _publish_plot(self):
        self._setup_figure()
        px, py, yaw, use_est = self._get_pose()
        kf_dx, kf_dy = self._get_kf_odom_offset()
        # Effective heading = static offset + dynamic yaw trim; pitch trim for tilt
        effective_heading_rad = self._heading_offset_rad + math.radians(self._yaw_trim_deg)
        pitch_trim_rad = math.radians(self._pitch_trim_deg)

        with self._lock:
            landmarks = list(self._landmarks)
            observations = list(self._observations)
            aircraft_data = self._aircraft_data
            demo = self._demo_mode

        # Demo mode: synthetic landmarks/obs
        if demo and not landmarks:
            landmarks = [
                {'id': i, 'x': px + 10 + i * 5, 'y': py + 5 + i * 3}
                for i in range(5)
            ]
        if demo and not observations:
            obs_map = {lm['id']: lm for lm in landmarks}
            observations = []
            for lid, lm in obs_map.items():
                xc, yc, zc = world_to_camera(
                    lm['x'], lm['y'], 0.0,
                    px + self._cam_offset_x, py + self._cam_offset_y, self._cam_height,
                    yaw, effective_heading_rad, pitch_trim_rad
                )
                uv = self._camera.project(xc, yc, zc)
                if uv:
                    observations.append({'id': lid, 'u': uv[0], 'v': uv[1]})

        # Camera center in world (body frame: X forward, Y left; rotate offset by yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        # Predicted camera: KF pose + offset (when use_est) or odom + offset (fallback)
        cam_x = px + c * self._cam_offset_x - s * self._cam_offset_y
        cam_y = py + s * self._cam_offset_x + c * self._cam_offset_y
        cam_z = self._cam_height
        # Actual camera: odom pose + offset (for World 2D predicted vs actual)
        odom_px = px - kf_dx if use_est else px
        odom_py = py - kf_dy if use_est else py
        actual_cam_x = odom_px + c * self._cam_offset_x - s * self._cam_offset_y
        actual_cam_y = odom_py + s * self._cam_offset_x + c * self._cam_offset_y

        obs_by_id = {o['id']: o for o in observations}
        pred_uvs = []
        actual_uvs = []
        residual_arrows = []
        pred_pts = []
        actual_pts = []
        lm_labels = []  # (mid_uv, lid) for text

        for lm in landmarks:
            lid = lm.get('id')
            lx, ly = lm['x'], lm['y']
            xc, yc, zc = world_to_camera(
                lx, ly, 0.0, cam_x, cam_y, cam_z, yaw, effective_heading_rad, pitch_trim_rad
            )
            uv = self._camera.project(xc, yc, zc)
            if uv:
                pred_uvs.append(uv)
                pred_pts.append((lx, ly, lid))
                obs = obs_by_id.get(lid) if lid is not None else None
                if obs:
                    ou, ov = obs['u'], obs['v']
                    actual_uvs.append((ou, ov))
                    actual_pts.append((lx, ly, lid))
                    residual_arrows.append(((uv[0], uv[1]), (ou - uv[0], ov - uv[1])))
                    mid_u, mid_v = (uv[0] + ou) / 2, (uv[1] + ov) / 2
                    lm_labels.append((mid_u, mid_v, str(lid) if lid is not None else '?'))
                else:
                    lm_labels.append((uv[0], uv[1], str(lid) if lid is not None else '?'))

        # Aircraft: trike-centric from state vectors; apply KF vs odom offset for projection
        aircraft_list = []
        if self._enable_aircraft and aircraft_data:
            alist = aircraft_data.get('aircraft') or []
            origin = aircraft_data.get('header', {}).get('origin_lla') or {}
            lat0 = origin.get('lat') or self._ref_lat
            lon0 = origin.get('lon') or self._ref_lon
            alt0 = origin.get('alt_m') or self._ref_alt

            for ac in alist:
                pose = ac.get('pose') or {}
                pos = pose.get('position') or {}
                ax = float(pos.get('x', 0))
                ay = float(pos.get('y', 0))
                az = float(pos.get('z', 0))
                dist = math.sqrt(ax * ax + ay * ay)
                if dist > self._aircraft_max_km * M_PER_KM:
                    continue
                if dist < self._aircraft_min_m and dist > 0:
                    continue
                dist_km = dist / M_PER_KM
                aircraft_list.append({
                    'x': ax, 'y': ay, 'z': az,
                    'icao': (ac.get('icao') or '').strip()[:6] or '??????',
                    'dist_km': dist_km,
                })

        # Aircraft: predicted (KF) vs actual (odom) - same style as landmarks
        # Aircraft (ax,ay) from state vectors are relative to odom trike; world = odom + (ax,ay)
        ac_pred_uvs = []
        ac_actual_uvs = []
        ac_residual_arrows = []
        ac_pred_only = []
        ac_world_pts = []
        ac_info = []  # (world_x, world_y, icao, dist_km)
        for ac in aircraft_list:
            ax, ay, az = ac['x'], ac['y'], ac['z']
            icao = ac.get('icao', '??????')
            dist_km = ac.get('dist_km', 0.0)
            ac_world_x = odom_px + ax
            ac_world_y = odom_py + ay
            ac_world_pts.append((ac_world_x, ac_world_y))
            ac_info.append((ac_world_x, ac_world_y, icao, dist_km))
            # Predicted: KF camera sees aircraft at (odom+ax) in world
            xc_p, yc_p, zc_p = world_to_camera_3d(
                ac_world_x, ac_world_y, self._ref_alt + az,
                cam_x, cam_y, cam_z, yaw, effective_heading_rad, pitch_trim_rad
            )
            uv_pred = self._camera.project(xc_p, yc_p, zc_p) if zc_p > 0.1 else None
            # Actual: odom camera sees aircraft at (odom+ax) in world
            xc_a, yc_a, zc_a = world_to_camera_3d(
                ac_world_x, ac_world_y, self._ref_alt + az,
                actual_cam_x, actual_cam_y, cam_z, yaw, effective_heading_rad, pitch_trim_rad
            )
            uv_actual = self._camera.project(xc_a, yc_a, zc_a) if zc_a > 0.1 else None
            if uv_pred and uv_actual:
                ac_pred_uvs.append(uv_pred)
                ac_actual_uvs.append(uv_actual)
                du = uv_actual[0] - uv_pred[0]
                dv = uv_actual[1] - uv_pred[1]
                ac_residual_arrows.append((uv_pred, (du, dv), icao, dist_km))
            elif uv_pred:
                ac_pred_only.append((uv_pred, icao, dist_km))

        # Draw
        self._ax_img.clear()
        self._ax_img.set_xlim(0, self._cam_w)
        self._ax_img.set_ylim(self._cam_h, 0)
        self._ax_img.set_aspect('equal')
        self._ax_img.set_xlabel('u (px)', fontsize=8)
        self._ax_img.set_ylabel('v (px)', fontsize=8)
        self._ax_img.set_title('Image plane (predicted vs actual)', fontsize=9)
        self._ax_img.grid(True, alpha=0.4)
        self._ax_img.set_facecolor('#1a1a1a')
        # FOV label: same camera model as World 2D HFoV cone
        hfov_deg = math.degrees(self._camera.hfov_rad())
        vfov_deg = math.degrees(self._camera.vfov_rad())
        self._ax_img.text(0.02, 0.98, 'HFoV %.1f°  VFoV %.1f°' % (hfov_deg, vfov_deg),
                          transform=self._ax_img.transAxes, fontsize=7, color=COLOR_INNOVATION,
                          va='top', ha='left', bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
        # Horizon line: v = cy + fy * tan(pitch); pitch 0 = horizon at vertical center
        v_horizon = self._cam_cy + self._cam_fy * math.tan(pitch_trim_rad)
        if 0 <= v_horizon <= self._cam_h:
            self._ax_img.axhline(v_horizon, color='#6a9bd8', linewidth=1.5, linestyle='--', alpha=0.8, zorder=3)

        if pred_uvs:
            us, vs = zip(*pred_uvs)
            self._ax_img.scatter(us, vs, c=COLOR_KF, s=40, marker='o', edgecolors=COLOR_KF_EDGE, linewidths=1, label='Predicted', zorder=5)
        if actual_uvs:
            us, vs = zip(*actual_uvs)
            self._ax_img.scatter(us, vs, c=COLOR_ACTUAL, s=50, marker='x', linewidths=2, label='Actual', zorder=6)
        for (pu, pv), (du, dv) in residual_arrows:
            self._ax_img.arrow(pu, pv, du, dv, head_width=4, head_length=4, fc=COLOR_INNOVATION, ec=COLOR_INNOVATION, linewidth=1.5, zorder=4)
        for (mu, mv, lbl) in lm_labels:
            self._ax_img.text(mu, mv, lbl, fontsize=7, color='white', ha='center', va='center', zorder=7)
        # Aircraft: predicted (orange) vs actual (blue) with innovation (cyan); info only at actual
        for uv_pred, dudv, icao, dist_km in ac_residual_arrows:
            du, dv = dudv[0], dudv[1]
            pu, pv = uv_pred[0], uv_pred[1]
            au, av = pu + du, pv + dv
            self._ax_img.plot(pu, pv, 'o', color=COLOR_KF, markersize=6, markeredgecolor=COLOR_KF_EDGE, markeredgewidth=1, zorder=6)
            self._ax_img.plot(au, av, 'x', color=COLOR_ACTUAL, markersize=8, markeredgewidth=2, zorder=6)
            if abs(du) > 1 or abs(dv) > 1:
                self._ax_img.arrow(pu, pv, du, dv, head_width=6, head_length=5, fc=COLOR_INNOVATION, ec=COLOR_INNOVATION, alpha=0.9, zorder=5)
            self._ax_img.text(au, av - 22, icao, fontsize=7, color=COLOR_ACTUAL, ha='center', va='top', zorder=7)
            self._ax_img.text(au, av + 18, '%.2fkm' % dist_km, fontsize=6, color='#aaa', ha='center', va='bottom', zorder=7)
        for uv_pred, icao, dist_km in ac_pred_only:
            pu, pv = uv_pred[0], uv_pred[1]
            self._ax_img.plot(pu, pv, 'o', color=COLOR_KF, markersize=5, alpha=0.7, zorder=5)

        # Image plane legend (blue=actual, orange=predicted, cyan=innovation, horizon)
        img_legend = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_KF, markeredgecolor=COLOR_KF_EDGE, markersize=8, label='Predicted'),
            Line2D([0], [0], marker='x', color=COLOR_ACTUAL, markersize=10, markeredgewidth=2, label='Actual'),
            Line2D([0], [0], color=COLOR_INNOVATION, linewidth=2, label='Innovation'),
            Line2D([0], [0], color='#6a9bd8', linewidth=2, linestyle='--', label='Horizon'),
        ]
        self._ax_img.legend(handles=img_legend, loc='upper right', fontsize=7)

        pose_label = '[estimated]' if use_est else '[odom]'
        self._ax_img.text(0.98, 0.02, pose_label, transform=self._ax_img.transAxes, fontsize=8, color='yellow', ha='right', va='bottom')
        self._frame_count += 1
        self._ax_img.text(0.98, 0.98, '#%d' % self._frame_count, transform=self._ax_img.transAxes, fontsize=8, color='white', ha='right', va='top')

        # World 2D view - centered on actual vehicle (odom)
        self._ax_world.clear()
        extent_m = self._world_extent_km * M_PER_KM * 0.5
        center_x, center_y = odom_px, odom_py
        self._ax_world.set_xlim(center_x - extent_m, center_x + extent_m)
        self._ax_world.set_ylim(center_y - extent_m, center_y + extent_m)
        self._ax_world.set_aspect('equal')
        self._ax_world.set_xlabel('East (m)', fontsize=8)
        self._ax_world.set_ylabel('North (m)', fontsize=8)
        self._ax_world.set_title('World 2D (%.1f km extent)' % self._world_extent_km, fontsize=9)
        self._ax_world.grid(True, alpha=0.4)
        self._ax_world.axhline(center_y, color='k', linewidth=0.5, alpha=0.3)
        self._ax_world.axvline(center_x, color='k', linewidth=0.5, alpha=0.3)
        self._ax_world.set_facecolor('#1a1a1a')

        # Vehicle: predicted (KF) vs actual (odom) when use_est
        self._ax_world.plot(odom_px, odom_py, 'o', color=COLOR_ACTUAL, markersize=12, markeredgecolor=COLOR_ACTUAL_EDGE, markeredgewidth=2, label='Vehicle (actual)', zorder=10)
        if use_est and (kf_dx != 0 or kf_dy != 0):
            self._ax_world.plot(px, py, 'o', color=COLOR_KF, markersize=10, markeredgecolor=COLOR_KF_EDGE, markeredgewidth=1.5, label='Vehicle (KF)', zorder=10)

        # Camera: predicted (KF) vs actual (odom)
        self._ax_world.plot(actual_cam_x, actual_cam_y, 'x', color=COLOR_ACTUAL, markersize=10, markeredgewidth=2.5, label='Camera (actual)', zorder=10)
        if use_est and (kf_dx != 0 or kf_dy != 0):
            self._ax_world.plot(cam_x, cam_y, 'D', color=COLOR_KF, markersize=8, markeredgecolor=COLOR_KF_EDGE, markeredgewidth=1.5, label='Camera (KF)', zorder=10)
            # Innovation: predicted → actual camera
            self._ax_world.arrow(cam_x, cam_y, actual_cam_x - cam_x, actual_cam_y - cam_y,
                                head_width=15, head_length=12, fc=COLOR_INNOVATION, ec=COLOR_INNOVATION, alpha=0.8, zorder=9)
        offset_mag = math.hypot(self._cam_offset_x, self._cam_offset_y)
        if offset_mag > 0.1:
            self._ax_world.plot([odom_px, actual_cam_x], [odom_py, actual_cam_y], color=COLOR_ACTUAL, linewidth=1, alpha=0.5, zorder=9)
            if use_est and (kf_dx != 0 or kf_dy != 0):
                self._ax_world.plot([px, cam_x], [py, cam_y], color=COLOR_KF, linewidth=1, alpha=0.5, zorder=9)

        # HFoV cone from actual camera (always)
        hfov = self._camera.hfov_rad()
        half = hfov / 2.0
        effective_yaw = yaw + effective_heading_rad
        cone_dist = min(extent_m * 0.3, 2000.0)
        left_ang = effective_yaw - half
        right_ang = effective_yaw + half
        hfov_deg = math.degrees(hfov)
        if self._yaw_zero_east:
            lx_a = actual_cam_x + cone_dist * math.cos(left_ang)
            ly_a = actual_cam_y + cone_dist * math.sin(left_ang)
            rx_a = actual_cam_x + cone_dist * math.cos(right_ang)
            ry_a = actual_cam_y + cone_dist * math.sin(right_ang)
            mid_x_a = actual_cam_x + cone_dist * 0.5 * math.cos(effective_yaw)
            mid_y_a = actual_cam_y + cone_dist * 0.5 * math.sin(effective_yaw)
        else:
            lx_a = actual_cam_x + cone_dist * math.sin(left_ang)
            ly_a = actual_cam_y + cone_dist * math.cos(left_ang)
            rx_a = actual_cam_x + cone_dist * math.sin(right_ang)
            ry_a = actual_cam_y + cone_dist * math.cos(right_ang)
            mid_x_a = actual_cam_x + cone_dist * 0.5 * math.sin(effective_yaw)
            mid_y_a = actual_cam_y + cone_dist * 0.5 * math.cos(effective_yaw)
        self._ax_world.fill([actual_cam_x, lx_a, rx_a, actual_cam_x], [actual_cam_y, ly_a, ry_a, actual_cam_y], alpha=0.2, color=COLOR_ACTUAL)
        self._ax_world.plot([actual_cam_x, lx_a], [actual_cam_y, ly_a], '-', color=COLOR_ACTUAL, alpha=0.6, linewidth=1)
        self._ax_world.plot([actual_cam_x, rx_a], [actual_cam_y, ry_a], '-', color=COLOR_ACTUAL, alpha=0.6, linewidth=1)
        self._ax_world.text(mid_x_a, mid_y_a, 'HFoV %.1f°' % hfov_deg, fontsize=7, color=COLOR_ACTUAL, ha='center')

        # HFoV cone from predicted (KF) camera (when use_est)
        if use_est and (kf_dx != 0 or kf_dy != 0):
            if self._yaw_zero_east:
                lx = cam_x + cone_dist * math.cos(left_ang)
                ly = cam_y + cone_dist * math.sin(left_ang)
                rx = cam_x + cone_dist * math.cos(right_ang)
                ry = cam_y + cone_dist * math.sin(right_ang)
                mid_x = cam_x + cone_dist * 0.5 * math.cos(effective_yaw)
                mid_y = cam_y + cone_dist * 0.5 * math.sin(effective_yaw)
            else:
                lx = cam_x + cone_dist * math.sin(left_ang)
                ly = cam_y + cone_dist * math.cos(left_ang)
                rx = cam_x + cone_dist * math.sin(right_ang)
                ry = cam_y + cone_dist * math.cos(right_ang)
                mid_x = cam_x + cone_dist * 0.5 * math.sin(effective_yaw)
                mid_y = cam_y + cone_dist * 0.5 * math.cos(effective_yaw)
            self._ax_world.fill([cam_x, lx, rx, cam_x], [cam_y, ly, ry, cam_y], alpha=0.15, color=COLOR_KF)
            self._ax_world.plot([cam_x, lx], [cam_y, ly], '-', color=COLOR_KF, alpha=0.5, linewidth=1)
            self._ax_world.plot([cam_x, rx], [cam_y, ry], '-', color=COLOR_KF, alpha=0.5, linewidth=1)
            self._ax_world.text(mid_x, mid_y, 'HFoV %.1f°' % hfov_deg, fontsize=7, color=COLOR_KF_EDGE, ha='center')

        # Landmarks in world (odom frame) with ID labels
        if pred_pts:
            lxs = [p[0] for p in pred_pts]
            lys = [p[1] for p in pred_pts]
            lids = [p[2] if len(p) > 2 else None for p in pred_pts]
            self._ax_world.scatter(lxs, lys, c=COLOR_LANDMARKS, s=25, marker='.', label='Landmarks', zorder=4)
            for (lx, ly, lid) in zip(lxs, lys, lids):
                if lid is not None:
                    self._ax_world.text(lx, ly + 15, str(lid), fontsize=7, color=COLOR_LANDMARKS, ha='center', va='bottom', zorder=6)

        # Aircraft in world (odom frame) with ICAO and range
        if ac_world_pts:
            axs, ays = zip(*ac_world_pts)
            self._ax_world.scatter(axs, ays, c=COLOR_AIRCRAFT, s=40, marker='^', edgecolors='#00897B', label='Aircraft', zorder=5)
        for (wx, wy, icao, dist_km) in ac_info:
            self._ax_world.text(wx, wy - 25, icao, fontsize=8, color=COLOR_AIRCRAFT, ha='center', va='top', zorder=6)
            self._ax_world.text(wx, wy + 5, '%.2fkm' % dist_km, fontsize=7, color='#888', ha='center', va='bottom', zorder=6)

        # World 2D legend (blue=actual, orange=KF, cyan=innovation)
        world_legend = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_ACTUAL, markeredgecolor=COLOR_ACTUAL_EDGE, markersize=10, label='Vehicle (actual)'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_KF, markeredgecolor=COLOR_KF_EDGE, markersize=8, label='Vehicle (KF)'),
            Line2D([0], [0], marker='x', color=COLOR_ACTUAL, markersize=10, markeredgewidth=2, label='Camera (actual)'),
            Line2D([0], [0], marker='D', color='w', markerfacecolor=COLOR_KF, markeredgecolor=COLOR_KF_EDGE, markersize=8, label='Camera (KF)'),
            Line2D([0], [0], color=COLOR_ACTUAL, linewidth=3, label='HFoV (actual)'),
            Line2D([0], [0], color=COLOR_KF, linewidth=3, label='HFoV (KF)'),
            Line2D([0], [0], color=COLOR_INNOVATION, linewidth=2, label='Innovation'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor=COLOR_LANDMARKS, markersize=6, linestyle='None', label='Landmarks'),
            Line2D([0], [0], marker='^', color='w', markerfacecolor=COLOR_AIRCRAFT, markeredgecolor='#00897B', markersize=8, label='Aircraft'),
        ]
        self._ax_world.legend(handles=world_legend, loc='upper right', fontsize=7)

        self._fig.canvas.draw()
        w, h = self._fig.canvas.get_width_height()
        raw = self._fig.canvas.tostring_rgb()
        if len(raw) != w * h * 3:
            return

        msg = Image()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = list(raw)
        self._image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkVOPlot2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
