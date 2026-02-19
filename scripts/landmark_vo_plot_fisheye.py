#!/usr/bin/env python3
"""
Spherical Fisheye Plot Node

Pure spherical fisheye (equidistant): center=zenith, edge=horizon, 180° FOV.
Single spherical projection—no pinhole FOV, no separate footprint.
Shows landmarks and aircraft (predicted vs actual) in an all-sky view.
Publishes on ~/fisheye_panorama (sensor_msgs/Image).

Subscribes to same topics as landmark_vo_plot_2d: odom, estimated_position,
landmarks, observations, aircraft_state_vectors.

Usage:
  ros2 launch deepgis_vehicles landmark_vo_plot_fisheye.launch.py
  ros2 run rqt_image_view rqt_image_view  # subscribe to /landmark_vo_plot_fisheye/fisheye_panorama
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
        plt.rcParams.update({'figure.facecolor': '#0a0a0f', 'axes.facecolor': '#0a0a0f'})
        return plt
    except (ImportError, AttributeError) as e:
        err = str(e)
        if 'numpy' in err.lower() or 'ARRAY_API' in err:
            print('Matplotlib/NumPy conflict. Fix: pip install "numpy<2"\n', file=sys.stderr)
        raise

import rclpy
from matplotlib.patches import Circle, Polygon, Rectangle
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Header

EARTH_R_M = 6371000.0
M_PER_KM = 1000.0

COLOR_ACTUAL = '#2196F3'
COLOR_ACTUAL_EDGE = '#1565C0'
COLOR_KF = '#FF9800'
COLOR_KF_EDGE = '#FFB74D'
COLOR_INNOVATION = '#00BCD4'
COLOR_LANDMARKS = '#9C27B0'
COLOR_AIRCRAFT = '#26A69A'


def lla_to_enu_m(lat_deg, lon_deg, alt_m, lat0_deg, lon0_deg, alt0_m):
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


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _radial_offset(u, v, cx, cy, dist_km, scale):
    """Apply log-distance radial offset from circumference inward. Farther aircraft pull toward center (zenith);
    horizon planes form concentric rings, reducing overlap. Center stays sparse."""
    if scale <= 0 or dist_km <= 0:
        return u, v
    r_proj = math.sqrt((u - cx) ** 2 + (v - cy) ** 2)
    if r_proj < 1e-6:
        return u, v
    offset = scale * math.log10(max(dist_km, 0.05) + 1.0)
    r_new = max(0.0, r_proj - offset)
    s = r_new / r_proj if r_proj > 1e-6 else 0.0
    return cx + (u - cx) * s, cy + (v - cy) * s


def world_to_camera(x_w, y_w, z_w, cam_x, cam_y, cam_z, yaw_rad,
                    heading_offset_rad=0.0, pitch_offset_rad=0.0):
    effective_yaw = yaw_rad + heading_offset_rad
    c, s = math.cos(-effective_yaw), math.sin(-effective_yaw)
    dx, dy, dz = x_w - cam_x, y_w - cam_y, z_w - cam_z
    lx_b = c * dx - s * dy
    ly_b = s * dx + c * dy
    lz_b = dz
    cp, sp = math.cos(-pitch_offset_rad), math.sin(-pitch_offset_rad)
    lx_bp = cp * lx_b + sp * lz_b
    lz_bp = -sp * lx_b + cp * lz_b
    x_cam = -ly_b
    y_cam = -lz_bp
    z_cam = lx_bp
    return (x_cam, y_cam, z_cam)


class PolarFisheyeProjection:
    """
    Pure spherical fisheye (equidistant projection).
    Center = zenith, edge = horizon. r = R_max * (zenith_angle / 90°).
    Camera frame: X right, Y down, Z forward. Up = -Y.
    """

    def __init__(self, size: int = 512):
        self.size = size
        self.cx = size / 2.0
        self.cy = size / 2.0
        self.radius_max = size / 2.0  # horizon = circle edge (whole sky)

    def project(self, x_cam: float, y_cam: float, z_cam: float):
        """Project 3D point in camera frame to (u, v). Returns None if behind horizon."""
        r = math.sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam)
        if r < 1e-9:
            return None
        # Up = -Y. Zenith angle = angle from straight up.
        cos_zenith = max(-1.0, min(1.0, -y_cam / r))
        zenith_angle_rad = math.acos(cos_zenith)
        zenith_angle_deg = math.degrees(zenith_angle_rad)
        # Azimuth: forward = +Z, angle in horizontal plane. phi = atan2(x, z)
        phi = math.atan2(x_cam, z_cam)
        # Equidistant: r_proj = radius_max * (zenith_angle / 90°)
        r_proj = self.radius_max * (zenith_angle_deg / 90.0)
        u = self.cx + r_proj * math.sin(phi)
        v = self.cy - r_proj * math.cos(phi)
        return (u, v)


class LandmarkVOPlotFisheyeNode(Node):
    def __init__(self):
        super().__init__('landmark_vo_plot_fisheye')

        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('estimated_position_topic', '/adsb/rtl_adsb_decoder_node/estimated_position')
        self.declare_parameter('landmarks_topic', '/vo/landmarks')
        self.declare_parameter('observations_topic', '/vo/landmark_observations')
        self.declare_parameter('mavros_local_frame', 'enu')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('image_size', 640)  # Square circular plot (astronomy style)
        self.declare_parameter('cam_offset_x', 0.0)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('cam_height', 1.2)
        self.declare_parameter('camera_heading_offset_deg', 0.0)
        self.declare_parameter('yaw_trim_deg', 0.0)
        self.declare_parameter('pitch_trim_deg', 0.0)
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('adsb_state_vectors_topic', '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors')
        self.declare_parameter('enable_aircraft', True)
        self.declare_parameter('aircraft_max_range_km', 8.0)
        self.declare_parameter('aircraft_min_range_m', 50.0)
        self.declare_parameter('aircraft_log_offset_scale', 12.0)
        self.declare_parameter('camera_hfov_deg', 65.2)
        self.declare_parameter('camera_vfov_deg', 54.2)
        self.declare_parameter('show_camera_footprint', True)
        self.declare_parameter('mavros_namespace', '/mavros')

        self._odom_topic = self.get_parameter('odom_topic').value
        self._est_pos_topic = self.get_parameter('estimated_position_topic').value
        self._landmarks_topic = self.get_parameter('landmarks_topic').value
        self._obs_topic = self.get_parameter('observations_topic').value
        self._adsb_topic = self.get_parameter('adsb_state_vectors_topic').value
        self._mavros_frame = self.get_parameter('mavros_local_frame').value
        self._demo_mode = self.get_parameter('demo_mode').value
        self._enable_aircraft = self.get_parameter('enable_aircraft').value
        self._aircraft_max_km = float(self.get_parameter('aircraft_max_range_km').value)
        self._aircraft_min_m = float(self.get_parameter('aircraft_min_range_m').value)
        self._aircraft_log_offset = float(self.get_parameter('aircraft_log_offset_scale').value)
        self._cam_hfov_deg = float(self.get_parameter('camera_hfov_deg').value)
        self._cam_vfov_deg = float(self.get_parameter('camera_vfov_deg').value)
        self._show_footprint = self.get_parameter('show_camera_footprint').value
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._img_size = int(self.get_parameter('image_size').value)
        self._cam_offset_x = float(self.get_parameter('cam_offset_x').value)
        self._cam_offset_y = float(self.get_parameter('cam_offset_y').value)
        self._cam_height = float(self.get_parameter('cam_height').value)
        self._heading_offset_rad = math.radians(float(self.get_parameter('camera_heading_offset_deg').value))
        self._yaw_trim_deg = float(self.get_parameter('yaw_trim_deg').value)
        self._pitch_trim_deg = float(self.get_parameter('pitch_trim_deg').value)

        self._projection = PolarFisheyeProjection(self._img_size)

        self._lock = threading.Lock()
        self._odom = None
        self._est_pos_json = None
        self._landmarks = []
        self._observations = []
        self._aircraft_data = None
        self._ref_lat = None
        self._ref_lon = None
        self._ref_alt = 0.0
        self._ref_set = False
        self._fig = None
        self._ax = None
        self._plt = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, qos)
        if self._est_pos_topic:
            self.create_subscription(String, self._est_pos_topic, self._est_pos_cb, 10)
        self.create_subscription(String, self._landmarks_topic, self._landmarks_cb, 10)
        self.create_subscription(String, self._obs_topic, self._observations_cb, 10)
        mavros_ns = self.get_parameter('mavros_namespace').value or '/mavros'
        self.create_subscription(NavSatFix, f'{mavros_ns}/global_position/raw/fix', self._fix_cb, qos)
        self.create_subscription(NavSatFix, f'{mavros_ns}/global_position/global', self._global_cb, qos)
        if self._enable_aircraft and self._adsb_topic:
            self.create_subscription(String, self._adsb_topic, self._aircraft_cb, 10)

        self._image_pub = self.create_publisher(Image, '~/fisheye_panorama', 10)
        self._timer = self.create_timer(1.0 / max(0.5, self._rate_hz), self._publish_cb)
        self.get_logger().info(
            'Landmark VO Fisheye: odom=%s, landmarks=%s, obs=%s, adsb=%s' % (
                self._odom_topic, self._landmarks_topic, self._obs_topic, self._adsb_topic)
        )

    def _odom_cb(self, msg):
        self._odom = msg

    def _est_pos_cb(self, msg):
        try:
            self._est_pos_json = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            pass

    def _landmarks_cb(self, msg):
        try:
            data = json.loads(msg.data or '{}')
            lst = data.get('landmarks')
            if isinstance(lst, list):
                with self._lock:
                    self._landmarks = [
                        {'id': lm.get('id'), 'x': float(lm.get('x', 0)), 'y': float(lm.get('y', 0))}
                        for lm in lst if isinstance(lm, dict)
                    ]
            else:
                with self._lock:
                    self._landmarks = []
        except json.JSONDecodeError:
            self._landmarks = []

    def _observations_cb(self, msg):
        try:
            data = json.loads(msg.data or '{}')
            lst = data.get('observations')
            if isinstance(lst, list):
                with self._lock:
                    self._observations = [
                        {'id': o.get('id'), 'u': float(o.get('u', 0)), 'v': float(o.get('v', 0))}
                        for o in lst if isinstance(o, dict)
                    ]
            else:
                with self._lock:
                    self._observations = []
        except json.JSONDecodeError:
            self._observations = []

    def _set_ref(self, lat, lon, alt):
        if not self._ref_set and not (math.isnan(lat) or math.isnan(lon)):
            self._ref_lat = lat
            self._ref_lon = lon
            self._ref_alt = alt if not math.isnan(alt) else 0.0
            self._ref_set = True
            self.get_logger().info('Ref: lat=%.6f lon=%.6f alt=%.1f' % (lat, lon, self._ref_alt))

    def _fix_cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        try:
            if hasattr(msg, 'status') and getattr(msg.status, 'status', -1) >= 0:
                self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))
                return
        except (AttributeError, TypeError):
            pass
        self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))

    def _global_cb(self, msg):
        if msg.latitude != 0 or msg.longitude != 0:
            self._set_ref(msg.latitude, msg.longitude, float(msg.altitude))

    def _aircraft_cb(self, msg):
        try:
            data = json.loads(msg.data or '{}')
            self._aircraft_data = data
        except json.JSONDecodeError:
            pass

    def _get_pose(self):
        with self._lock:
            odom, est, ref_set = self._odom, self._est_pos_json, self._ref_set
            ref_lat, ref_lon, ref_alt = self._ref_lat, self._ref_lon, self._ref_alt

        if not odom:
            return 0.0, 0.0, 0.0, False
        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        if (self._mavros_frame or 'enu').lower() == 'ned':
            ox, oy = p.y, p.x
        else:
            ox, oy = p.x, p.y
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)

        if not self._est_pos_topic or not est or not ref_set:
            return ox, oy, yaw, False
        has_kf = est.get('kf_lat') is not None and est.get('kf_lon') is not None
        lat = est.get('kf_lat') if has_kf else est.get('lat') or est.get('raw_lat')
        lon = est.get('kf_lon') if has_kf else est.get('lon') or est.get('raw_lon')
        if lat is None or lon is None:
            return ox, oy, yaw, False
        alt = float(est.get('altitude', ref_alt) or ref_alt)
        ex, ey, _ = lla_to_enu_m(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        return ex, ey, yaw, True

    def _get_kf_odom_offset(self):
        with self._lock:
            odom, est, ref_set = self._odom, self._est_pos_json, self._ref_set
        if not odom or not self._est_pos_topic or not est or not ref_set:
            return 0.0, 0.0
        if est.get('kf_lat') is None or est.get('kf_lon') is None:
            return 0.0, 0.0
        lat, lon = est['kf_lat'], est['kf_lon']
        alt = float(est.get('altitude', self._ref_alt) or self._ref_alt)
        kx, ky, _ = lla_to_enu_m(lat, lon, alt, self._ref_lat, self._ref_lon, self._ref_alt)
        p = odom.pose.pose.position
        ox, oy = (p.y, p.x) if (self._mavros_frame or 'enu').lower() == 'ned' else (p.x, p.y)
        return kx - ox, ky - oy

    def _publish_cb(self):
        self._plt = self._plt or _import_matplotlib()
        plt = self._plt
        px, py, yaw, use_est = self._get_pose()
        kf_dx, kf_dy = self._get_kf_odom_offset()
        effective_heading = self._heading_offset_rad + math.radians(self._yaw_trim_deg)
        pitch_trim = math.radians(self._pitch_trim_deg)

        with self._lock:
            landmarks = list(self._landmarks)
            observations = list(self._observations)
            aircraft_data = self._aircraft_data
            demo = self._demo_mode
            ref_lat, ref_lon, ref_alt = self._ref_lat, self._ref_lon, self._ref_alt

        if not self._ref_set and not landmarks:
            return

        if not self._ref_set:
            return

        c, s = math.cos(yaw), math.sin(yaw)
        cam_x = px + c * self._cam_offset_x - s * self._cam_offset_y
        cam_y = py + s * self._cam_offset_x + c * self._cam_offset_y
        cam_z = self._cam_height
        odom_px = px - kf_dx if use_est else px
        odom_py = py - kf_dy if use_est else py
        actual_cam_x = odom_px + c * self._cam_offset_x - s * self._cam_offset_y
        actual_cam_y = odom_py + s * self._cam_offset_x + c * self._cam_offset_y

        if demo and not landmarks:
            landmarks = [{'id': i, 'x': px + 20 + i * 10, 'y': py + 15 + i * 8} for i in range(5)]

        obs_by_id = {o['id']: o for o in observations}
        proj = self._projection

        # Landmarks: predicted (KF) vs actual (odom) in spherical view
        lm_pred_uv = []
        lm_actual_uv = []
        lm_residuals = []
        lm_ids = []

        for lm in landmarks:
            lid = lm.get('id')
            lx, ly = lm['x'], lm['y']
            xc_p, yc_p, zc_p = world_to_camera(lx, ly, 0.0, cam_x, cam_y, cam_z, yaw, effective_heading, pitch_trim)
            uv_pred = proj.project(xc_p, yc_p, zc_p)
            xc_a, yc_a, zc_a = world_to_camera(lx, ly, 0.0, actual_cam_x, actual_cam_y, cam_z, yaw, effective_heading, pitch_trim)
            uv_actual = proj.project(xc_a, yc_a, zc_a)
            if uv_pred:
                lm_pred_uv.append(uv_pred)
                lm_ids.append(lid)
                if uv_actual:
                    lm_actual_uv.append(uv_actual)
                    lm_residuals.append((uv_pred, (uv_actual[0] - uv_pred[0], uv_actual[1] - uv_pred[1])))

        # Aircraft
        aircraft_list = []
        if self._enable_aircraft and aircraft_data:
            alist = aircraft_data.get('aircraft') or []
            origin = aircraft_data.get('header', {}).get('origin_lla') or {}
            ac_origin_alt = float(origin.get('alt_m', ref_alt or 0.0))
            for ac in alist:
                pos = (ac.get('pose') or {}).get('position') or {}
                ax, ay, az = float(pos.get('x', 0)), float(pos.get('y', 0)), float(pos.get('z', 0))
                dist = math.sqrt(ax * ax + ay * ay)
                aircraft_list.append({
                    'x': ax, 'y': ay, 'z': az,
                    'alt_m': ac_origin_alt + az,
                    'icao': (ac.get('icao') or '').strip()[:6] or '??????',
                    'dist_km': dist / M_PER_KM,
                })

        ac_pred_uv = []
        ac_actual_uv = []
        ac_residuals = []
        ac_info = []

        cx, cy = self._img_size / 2.0, self._img_size / 2.0
        log_scale = self._aircraft_log_offset
        for ac in aircraft_list:
            ax, ay, az = ac['x'], ac['y'], ac['z']
            ac_alt = ac.get('alt_m', ref_alt + az)
            dist_km = ac['dist_km']
            ac_wx, ac_wy = odom_px + ax, odom_py + ay
            xc_p, yc_p, zc_p = world_to_camera(ac_wx, ac_wy, ac_alt, cam_x, cam_y, cam_z, yaw, effective_heading, pitch_trim)
            xc_a, yc_a, zc_a = world_to_camera(ac_wx, ac_wy, ac_alt, actual_cam_x, actual_cam_y, cam_z, yaw, effective_heading, pitch_trim)
            uv_pred = proj.project(xc_p, yc_p, zc_p)
            uv_actual = proj.project(xc_a, yc_a, zc_a)
            if uv_pred:
                pu, pv = _radial_offset(uv_pred[0], uv_pred[1], cx, cy, dist_km, log_scale)
                ac_pred_uv.append((pu, pv))
                ac_info.append((pu, pv, ac['icao'], dist_km))
                if uv_actual:
                    au, av = _radial_offset(uv_actual[0], uv_actual[1], cx, cy, dist_km, log_scale)
                    ac_actual_uv.append((au, av))
                    ac_residuals.append(((pu, pv), (au - pu, av - pv), ac['icao'], dist_km))

        # Draw: Circular polar plot (astronomy all-sky style)
        if self._fig is None:
            self._fig, self._ax = plt.subplots(1, 1, figsize=(self._img_size / 80, self._img_size / 80), dpi=80)
            self._ax.set_xlim(0, self._img_size)
            self._ax.set_ylim(self._img_size, 0)
            self._ax.set_aspect('equal')
            self._fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
            self._fig.patch.set_facecolor('#0a0a0f')

        ax = self._ax
        ax.clear()
        ax.set_facecolor('#0a0a0f')
        ax.set_xlim(0, self._img_size)
        ax.set_ylim(self._img_size, 0)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_position([0, 0, 1, 1])

        # Dark fill for entire axes (eliminates white corners)
        bg = Rectangle((0, 0), self._img_size, self._img_size, facecolor='#0a0a0f', edgecolor='none', zorder=0)
        ax.add_patch(bg)

        # Circular boundary (horizon) + fill - astronomy all-sky style (full radius)
        cx, cy = self._img_size / 2.0, self._img_size / 2.0
        R = self._img_size / 2.0
        circle = Circle((cx, cy), R, facecolor='#0d0d12', edgecolor='#3a3a4a', linewidth=2)
        ax.add_patch(circle)
        # Camera footprint (image-plane FOV on night sky)
        if self._show_footprint and self._cam_hfov_deg > 0 and self._cam_vfov_deg > 0:
            hfov2 = math.radians(self._cam_hfov_deg / 2.0)
            vfov2 = math.radians(self._cam_vfov_deg / 2.0)
            zenith_top = 90.0 - math.degrees(vfov2)
            r_outer = R * max(0.0, min(zenith_top, 90.0)) / 90.0
            r_inner = R
            theta_center = math.radians(90.0)  # Fwd at top (phi=0)
            theta1, theta2 = theta_center - hfov2, theta_center + hfov2
            n = 24
            outer_arc = [(cx + r_outer * math.cos(t), cy - r_outer * math.sin(t))
                         for t in [theta1 + (theta2 - theta1) * i / n for i in range(n + 1)]]
            inner_arc = [(cx + r_inner * math.cos(t), cy - r_inner * math.sin(t))
                         for t in [theta2 - (theta2 - theta1) * i / n for i in range(n + 1)]]
            verts = outer_arc + inner_arc
            footprint = Polygon(verts, fill=True, facecolor='#2a5a9a', edgecolor='#6ab4f8',
                               linewidth=2, alpha=0.5, zorder=5)
            ax.add_patch(footprint)
        # Cardinal marks at horizon (Fwd=phi 0 = top, match projection)
        label_offset = min(12, R * 0.04)
        for dx, dy, lbl in [(0, 1, 'Fwd'), (1, 0, 'R'), (0, -1, 'Back'), (-1, 0, 'L')]:
            ax.plot(cx + dx * R, cy - dy * R, 'o', color='#4a4a5a', markersize=3)
            ax.text(cx + dx * (R + label_offset), cy - dy * (R + label_offset), lbl, fontsize=7, color='#6a6a7a', ha='center', va='center')

        # Landmarks: predicted (orange) vs actual (blue)
        if lm_pred_uv:
            us, vs = zip(*lm_pred_uv)
            ax.scatter(us, vs, c=COLOR_KF, s=30, marker='o', edgecolors=COLOR_KF_EDGE, linewidths=1, zorder=5)
        for (pu, pv), (du, dv) in lm_residuals:
            au, av = pu + du, pv + dv
            ax.plot(au, av, 'x', color=COLOR_ACTUAL, markersize=8, markeredgewidth=2, zorder=6)
            if abs(du) > 2 or abs(dv) > 2:
                ax.arrow(pu, pv, du, dv, head_width=4, head_length=3, fc=COLOR_INNOVATION, ec=COLOR_INNOVATION, alpha=0.8, zorder=4)
        for i, (u, v) in enumerate(lm_pred_uv):
            if i < len(lm_ids) and lm_ids[i] is not None:
                ax.text(u, v - 8, str(lm_ids[i]), fontsize=6, color=COLOR_LANDMARKS, ha='center', va='top', zorder=7)

        # Aircraft
        for (pu, pv), (du, dv), icao, dist_km in ac_residuals:
            au, av = pu + du, pv + dv
            ax.plot(pu, pv, 'o', color=COLOR_KF, markersize=6, markeredgecolor=COLOR_KF_EDGE, zorder=5)
            ax.plot(au, av, 'x', color=COLOR_ACTUAL, markersize=8, markeredgewidth=2, zorder=6)
            if abs(du) > 2 or abs(dv) > 2:
                ax.arrow(pu, pv, du, dv, head_width=4, head_length=3, fc=COLOR_INNOVATION, ec=COLOR_INNOVATION, alpha=0.8, zorder=4)
            ax.text(au, av - 10, icao, fontsize=7, color=COLOR_AIRCRAFT, ha='center', va='top', zorder=7)
            ax.text(au, av + 8, '%.2fkm' % dist_km, fontsize=6, color='#888', ha='center', va='bottom', zorder=7)
        for (u, v, icao, dist_km) in ac_info:
            if not any(r[2] == icao for r in ac_residuals):
                ax.plot(u, v, 'o', color=COLOR_KF, markersize=5, alpha=0.8, zorder=5)
                ax.text(u, v - 8, icao, fontsize=7, color=COLOR_AIRCRAFT, ha='center', va='top', zorder=7)
                ax.text(u, v + 6, '%.2fkm' % dist_km, fontsize=6, color='#888', ha='center', va='bottom', zorder=7)

        # Title - compact, top inside circle
        ax.text(0.5, 0.02, '180° All-Sky | O=Pred X=Actual', transform=ax.transAxes,
                fontsize=8, color='#6a6a7a', ha='center', va='bottom')

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
    node = LandmarkVOPlotFisheyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
