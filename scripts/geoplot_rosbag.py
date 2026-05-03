#!/usr/bin/env python3
"""
Plot the Earth Rover trike path from a rosbag and emit time-series + CSV.

Reads NavSatFix on ``--gps-topic`` (default ``/mavros/global_position/raw/fix``)
plus optional companion topics (gps velocity, satellites, GPSRAW fix quality)
and writes a small report directory:

  <bag_parent>/<bag>_geoplot/
    path_local.png        # equirectangular meters, equal aspect, color-by-speed
    path_geo.png          # raw lat/lon scatter
    altitude.png          # altitude vs time
    speed.png             # ground speed vs time
    satellites.png        # satellite count + EPH (if available) vs time
    summary.png           # 2x2 dashboard
    path.csv              # t_unix, t_rel_s, lat, lon, alt, speed, ... per fix
    report.txt            # bag/path/quality summary

Designed so future overlays (laser distance, spectrometer cluster ids, ADS-B
aircraft tracks, etc.) can be added by aligning their sample times onto the
GPS fix timeline (see ``align_to``) and dropping a new layer onto
``path_local.png`` / ``summary.png``.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 ~/earth-rover/scripts/geoplot_rosbag.py \\
      --bag ~/earth-rover-bags/earth_rover_20260502_083404
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# --- sibling import (scripts/ is not a package) ------------------------------
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

try:
    from analyze_spectrometer_rosbag import (
        load_spectra_from_bag,
        open_bag_reader,
        resolve_topic_name,
    )
except ImportError as exc:
    print(f'Error importing sibling helpers: {exc}', file=sys.stderr)
    print(
        'Expected analyze_spectrometer_rosbag.py next to this script.',
        file=sys.stderr,
    )
    sys.exit(1)

# spectra-clusters overlay needs the cluster compute pipeline; loaded lazily
# because importing it pulls in sklearn/umap/hdbscan.
def _import_cluster_pipeline():
    try:
        from cluster_spectrometer_rosbag import compute_clusters_and_nmf  # noqa: WPS433
        return compute_clusters_and_nmf
    except ImportError as exc:
        raise SystemExit(
            f'Error importing cluster_spectrometer_rosbag: {exc}\n'
            'Install with: pip install --user scikit-learn umap-learn hdbscan'
        )

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    print(f'Error: {exc}', file=sys.stderr)
    print('Source ROS 2 first, e.g. `source /opt/ros/jazzy/setup.bash`.', file=sys.stderr)
    sys.exit(1)


# ---- geometry ---------------------------------------------------------------

# WGS84 equatorial radius — equirectangular projection is fine for the
# few-kilometer scales the trike covers in a single bag.
_R_EARTH_M = 6378137.0


def latlon_to_local_xy(
    lat: np.ndarray,
    lon: np.ndarray,
    lat0: Optional[float] = None,
    lon0: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Equirectangular projection centered on (lat0, lon0)."""
    if lat0 is None:
        lat0 = float(np.median(lat))
    if lon0 is None:
        lon0 = float(np.median(lon))
    cos_lat0 = math.cos(math.radians(lat0))
    x = np.deg2rad(lon - lon0) * _R_EARTH_M * cos_lat0
    y = np.deg2rad(lat - lat0) * _R_EARTH_M
    return x.astype(np.float64), y.astype(np.float64), lat0, lon0


def haversine_path_length(lat: np.ndarray, lon: np.ndarray) -> float:
    """Total path length in meters via cumulative haversine."""
    if lat.size < 2:
        return 0.0
    lat_r = np.deg2rad(lat)
    lon_r = np.deg2rad(lon)
    dlat = np.diff(lat_r)
    dlon = np.diff(lon_r)
    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat_r[:-1]) * np.cos(lat_r[1:]) * np.sin(dlon / 2.0) ** 2
    d = 2.0 * _R_EARTH_M * np.arcsin(np.sqrt(np.clip(a, 0.0, 1.0)))
    return float(d.sum())


def align_to(
    target_t: np.ndarray,
    src_t: np.ndarray,
    src_v: np.ndarray,
    mode: str = 'linear',
) -> np.ndarray:
    """Align ``src_v`` (sampled at ``src_t``) onto ``target_t``.

    ``mode`` is ``'linear'`` (np.interp) or ``'nearest'``. Returns NaN outside
    the source range. Future overlay loaders should produce ``(src_t, src_v)``
    and call this to attach data to GPS fixes.
    """
    if src_t.size == 0:
        return np.full(target_t.shape, np.nan, dtype=np.float64)
    order = np.argsort(src_t)
    st = src_t[order].astype(np.float64)
    sv = src_v[order].astype(np.float64)
    if mode == 'linear':
        return np.interp(target_t, st, sv, left=np.nan, right=np.nan)
    idx = np.searchsorted(st, target_t)
    idx = np.clip(idx, 1, len(st) - 1)
    use_left = (target_t - st[idx - 1]) <= (st[idx] - target_t)
    chosen = np.where(use_left, idx - 1, idx)
    out = sv[chosen]
    out[(target_t < st[0]) | (target_t > st[-1])] = np.nan
    return out


# ---- containers -------------------------------------------------------------


@dataclass
class GpsTrack:
    t_ns: np.ndarray  # (N,) header.stamp in ns since epoch
    lat: np.ndarray   # (N,) deg
    lon: np.ndarray   # (N,) deg
    alt: np.ndarray   # (N,) m
    nav_status: np.ndarray  # (N,) NavSatStatus.status (-1..2)
    cov_xy: np.ndarray      # (N,) sqrt(cov[0,0] + cov[4,4]) when available, else NaN

    def __len__(self) -> int:
        return int(self.t_ns.size)

    @property
    def t_s(self) -> np.ndarray:
        if self.t_ns.size == 0:
            return self.t_ns.astype(np.float64)
        return (self.t_ns - self.t_ns[0]).astype(np.float64) * 1e-9


@dataclass
class TimeSeries:
    t_ns: np.ndarray
    v: np.ndarray
    label: str = ''
    units: str = ''


@dataclass
class GpsExtras:
    speed: Optional[TimeSeries] = None        # ground speed (m/s)
    vspeed: Optional[TimeSeries] = None       # vertical speed (m/s, downward)
    satellites: Optional[TimeSeries] = None   # satellite count
    fix_type: Optional[TimeSeries] = None     # 0..6 from GPSRAW
    eph_m: Optional[TimeSeries] = None        # horizontal pos uncertainty (m)
    extras: Dict[str, TimeSeries] = field(default_factory=dict)


# ---- bag reading ------------------------------------------------------------


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _decimate(items: List, stride: int) -> List:
    if stride <= 1:
        return items
    return items[::stride]


def read_topics(
    bag_path: Path,
    requested: Dict[str, Optional[str]],
    expected_types: Dict[str, str],
) -> Tuple[Dict[str, List], Dict[str, str]]:
    """Single-pass read of the requested topics.

    ``requested`` maps caller key → user-supplied topic name (None to skip).
    ``expected_types`` maps caller key → ROS message type (used to load the
    deserializer class). Returns ``(records_by_key, resolved_topic_by_key)``.
    """
    reader, storage_id, storage_uri = open_bag_reader(bag_path)
    print(f'Opened bag: uri={storage_uri!r} storage_id={storage_id!r}')

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    resolved: Dict[str, str] = {}
    msg_classes: Dict[str, type] = {}
    keep_topics: List[str] = []

    for key, requested_name in requested.items():
        if requested_name is None:
            continue
        actual = resolve_topic_name(type_map, requested_name)
        if actual is None:
            print(
                f'  [skip] {key}: requested {requested_name!r} not in bag',
                file=sys.stderr,
            )
            continue
        actual_type = type_map[actual]
        expected = expected_types[key]
        if expected not in actual_type:
            print(
                f'  [warn] {key}: bag has {actual_type!r}, expected {expected!r}',
                file=sys.stderr,
            )
        try:
            msg_classes[key] = get_message(actual_type)
        except Exception as exc:
            print(f'  [skip] {key}: cannot load {actual_type!r}: {exc}', file=sys.stderr)
            continue
        resolved[key] = actual
        keep_topics.append(actual)
        print(f'  {key}: {actual} ({actual_type})')

    if not keep_topics:
        raise RuntimeError('None of the requested topics were found.')

    reader.set_filter(rosbag2_py.StorageFilter(topics=keep_topics))

    topic_to_key = {v: k for k, v in resolved.items()}
    records: Dict[str, List] = {k: [] for k in resolved}

    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        key = topic_to_key.get(topic)
        if key is None:
            continue
        msg = deserialize_message(data, msg_classes[key])
        records[key].append((int(t_rec), msg))

    return records, resolved


def parse_navsatfix(records: List, stride: int) -> GpsTrack:
    items = _decimate(records, stride)
    if not items:
        return GpsTrack(
            t_ns=np.array([], dtype=np.int64),
            lat=np.array([]), lon=np.array([]), alt=np.array([]),
            nav_status=np.array([], dtype=np.int8),
            cov_xy=np.array([]),
        )
    t_ns = np.empty(len(items), dtype=np.int64)
    lat = np.empty(len(items), dtype=np.float64)
    lon = np.empty(len(items), dtype=np.float64)
    alt = np.empty(len(items), dtype=np.float64)
    status = np.empty(len(items), dtype=np.int8)
    cov_xy = np.empty(len(items), dtype=np.float64)
    for i, (t_rec, msg) in enumerate(items):
        t_ns[i] = _stamp_to_ns(msg.header.stamp) or int(t_rec)
        lat[i] = float(msg.latitude)
        lon[i] = float(msg.longitude)
        alt[i] = float(msg.altitude)
        status[i] = int(getattr(msg.status, 'status', -1))
        cov = getattr(msg, 'position_covariance', None)
        if cov is not None and len(cov) >= 5:
            cov_xy[i] = float(math.sqrt(max(0.0, float(cov[0]) + float(cov[4]))))
        else:
            cov_xy[i] = float('nan')
    valid = (np.abs(lat) > 1e-6) & (np.abs(lon) > 1e-6) & np.isfinite(lat) & np.isfinite(lon)
    if not valid.any():
        return GpsTrack(
            t_ns=np.array([], dtype=np.int64),
            lat=np.array([]), lon=np.array([]), alt=np.array([]),
            nav_status=np.array([], dtype=np.int8),
            cov_xy=np.array([]),
        )
    return GpsTrack(
        t_ns=t_ns[valid],
        lat=lat[valid], lon=lon[valid], alt=alt[valid],
        nav_status=status[valid], cov_xy=cov_xy[valid],
    )


def parse_twist_speed(records: List) -> Tuple[TimeSeries, TimeSeries]:
    if not records:
        return (TimeSeries(np.array([], dtype=np.int64), np.array([]), 'speed', 'm/s'),
                TimeSeries(np.array([], dtype=np.int64), np.array([]), 'vspeed', 'm/s'))
    t_ns = np.empty(len(records), dtype=np.int64)
    speed = np.empty(len(records), dtype=np.float64)
    vspeed = np.empty(len(records), dtype=np.float64)
    for i, (t_rec, msg) in enumerate(records):
        t_ns[i] = _stamp_to_ns(msg.header.stamp) or int(t_rec)
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        vz = float(msg.twist.linear.z)
        speed[i] = math.hypot(vx, vy)
        vspeed[i] = vz
    return (TimeSeries(t_ns, speed, 'speed', 'm/s'),
            TimeSeries(t_ns, vspeed, 'vspeed', 'm/s'))


def parse_uint32(records: List, label: str) -> TimeSeries:
    if not records:
        return TimeSeries(np.array([], dtype=np.int64), np.array([]), label, '')
    t_ns = np.array([int(t) for (t, _) in records], dtype=np.int64)
    v = np.array([float(getattr(m, 'data', 0)) for (_, m) in records], dtype=np.float64)
    return TimeSeries(t_ns, v, label, '')


def parse_scalar(records: List, label: str, units: str = '') -> TimeSeries:
    """Generic ``.data`` scalar parser (Float32/Float64/Int*/UInt*)."""
    if not records:
        return TimeSeries(np.array([], dtype=np.int64), np.array([]), label, units)
    t_ns = np.array([int(t) for (t, _) in records], dtype=np.int64)
    v = np.array(
        [float(getattr(m, 'data', float('nan'))) for (_, m) in records],
        dtype=np.float64,
    )
    return TimeSeries(t_ns, v, label, units)


def parse_gpsraw(records: List) -> Tuple[TimeSeries, TimeSeries]:
    if not records:
        return (TimeSeries(np.array([], dtype=np.int64), np.array([]), 'fix_type', ''),
                TimeSeries(np.array([], dtype=np.int64), np.array([]), 'eph', 'm'))
    t_ns = np.empty(len(records), dtype=np.int64)
    fix_type = np.empty(len(records), dtype=np.float64)
    eph_m = np.empty(len(records), dtype=np.float64)
    for i, (t_rec, msg) in enumerate(records):
        stamp = getattr(msg, 'header', None)
        t_ns[i] = _stamp_to_ns(stamp.stamp) if stamp is not None else int(t_rec)
        fix_type[i] = float(getattr(msg, 'fix_type', 0))
        # GPSRAW.eph is in cm (per MAVLink GPS_RAW_INT scaling kept by mavros).
        raw_eph = getattr(msg, 'eph', 0.0)
        eph_m[i] = float(raw_eph) / 100.0 if raw_eph not in (None, 0) else float('nan')
    return (TimeSeries(t_ns, fix_type, 'fix_type', ''),
            TimeSeries(t_ns, eph_m, 'eph', 'm'))


# ---- plotting ---------------------------------------------------------------


_FIX_TYPE_LABELS = {
    0: 'NO_FIX', 1: 'NO_FIX', 2: '2D', 3: '3D',
    4: 'DGPS', 5: 'RTK_FLOAT', 6: 'RTK_FIXED',
}


def _select_color_values(
    track: GpsTrack, extras: GpsExtras, color_by: str,
) -> Tuple[np.ndarray, str]:
    if color_by == 'time':
        return track.t_s, 'Time (s from start)'
    if color_by == 'altitude':
        return track.alt, 'Altitude (m)'
    if color_by == 'satellites' and extras.satellites is not None and extras.satellites.t_ns.size:
        return align_to(track.t_ns.astype(np.float64),
                        extras.satellites.t_ns.astype(np.float64),
                        extras.satellites.v, mode='nearest'), 'Satellites'
    if color_by == 'fix_type' and extras.fix_type is not None and extras.fix_type.t_ns.size:
        return align_to(track.t_ns.astype(np.float64),
                        extras.fix_type.t_ns.astype(np.float64),
                        extras.fix_type.v, mode='nearest'), 'GPS fix_type'
    if color_by == 'speed' and extras.speed is not None and extras.speed.t_ns.size:
        return align_to(track.t_ns.astype(np.float64),
                        extras.speed.t_ns.astype(np.float64),
                        extras.speed.v, mode='linear'), 'Speed (m/s)'
    return track.t_s, 'Time (s from start)'


def plot_path_local(
    ax,
    track: GpsTrack,
    color_vals: np.ndarray,
    color_label: str,
    cmap: str,
    lat0: Optional[float] = None,
    lon0: Optional[float] = None,
    title: Optional[str] = None,
) -> Tuple[float, float]:
    x, y, lat0_used, lon0_used = latlon_to_local_xy(track.lat, track.lon, lat0, lon0)
    ax.plot(x, y, color='#444', lw=0.6, alpha=0.6, zorder=1)
    nan_mask = np.isnan(color_vals)
    if nan_mask.any():
        ax.scatter(
            x[nan_mask], y[nan_mask], s=8, c='lightgray', alpha=0.55,
            zorder=2, label='no data',
        )
    valid = ~nan_mask
    sc = None
    if valid.any():
        sc = ax.scatter(
            x[valid], y[valid], c=color_vals[valid], cmap=cmap, s=10, zorder=3,
        )
    ax.scatter([x[0]], [y[0]], marker='o', s=70, facecolor='none',
               edgecolor='lime', lw=1.6, label='start', zorder=4)
    ax.scatter([x[-1]], [y[-1]], marker='X', s=70, color='red',
               label='end', zorder=4)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title(
        title or f'Path (local, equirectangular)  origin=({lat0_used:.6f}, {lon0_used:.6f})'
    )
    if sc is not None:
        cb = plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
        cb.set_label(color_label)
    return lat0_used, lon0_used


def plot_path_clusters(
    ax,
    track: GpsTrack,
    cluster_ids: np.ndarray,
    lat0: float,
    lon0: float,
    title: str = 'Path colored by spectra cluster',
) -> None:
    """Discrete categorical coloring of the path by cluster id (NaN = no spectra)."""
    x, y, _, _ = latlon_to_local_xy(track.lat, track.lon, lat0, lon0)
    ax.plot(x, y, color='#444', lw=0.4, alpha=0.5, zorder=1)
    nan_mask = np.isnan(cluster_ids)
    if nan_mask.any():
        ax.scatter(
            x[nan_mask], y[nan_mask], s=6, c='lightgray', alpha=0.55,
            zorder=2, label='no spectra',
        )
    cmap = plt.get_cmap('tab20')
    ids = cluster_ids[~nan_mask].astype(int) if (~nan_mask).any() else np.array([], dtype=int)
    for lab in np.unique(ids):
        m_full = (~nan_mask) & (cluster_ids == lab)
        if not m_full.any():
            continue
        if lab == -1:
            color = '#999'
            label = f'noise (n={int(m_full.sum())})'
        else:
            color = cmap(int(lab) % 20)
            label = f'c{int(lab)} (n={int(m_full.sum())})'
        ax.scatter(x[m_full], y[m_full], c=[color], s=12, zorder=3, label=label)
    ax.scatter([x[0]], [y[0]], marker='o', s=80, facecolor='none',
               edgecolor='lime', lw=1.6, zorder=4)
    ax.scatter([x[-1]], [y[-1]], marker='X', s=80, color='red', zorder=4)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=7, ncol=2, framealpha=0.85)
    ax.set_title(title)


def plot_path_geo(ax, track: GpsTrack, color_vals: np.ndarray,
                  color_label: str, cmap: str) -> None:
    ax.plot(track.lon, track.lat, color='#444', lw=0.6, alpha=0.6, zorder=1)
    sc = ax.scatter(track.lon, track.lat, c=color_vals, cmap=cmap, s=10, zorder=2)
    ax.scatter([track.lon[0]], [track.lat[0]], marker='o', s=70, facecolor='none',
               edgecolor='lime', lw=1.6, label='start', zorder=3)
    ax.scatter([track.lon[-1]], [track.lat[-1]], marker='X', s=70, color='red',
               label='end', zorder=3)
    ax.set_xlabel('Longitude (deg)')
    ax.set_ylabel('Latitude (deg)')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title('Path (geographic)')
    cb = plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
    cb.set_label(color_label)


def plot_altitude(ax, track: GpsTrack) -> None:
    ax.plot(track.t_s, track.alt, lw=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title('Altitude vs time (raw GPS)')
    ax.grid(alpha=0.3)


def plot_speed(ax, track: GpsTrack, speed_aligned: np.ndarray) -> None:
    if np.all(np.isnan(speed_aligned)):
        ax.text(0.5, 0.5, 'no gps_vel topic', ha='center', va='center',
                transform=ax.transAxes)
        return
    ax.plot(track.t_s, speed_aligned, lw=1.0, color='tab:purple')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Ground speed vs time')
    ax.grid(alpha=0.3)


def plot_satellites(ax, track: GpsTrack, extras: GpsExtras) -> None:
    drew = False
    if extras.satellites is not None and extras.satellites.t_ns.size:
        sats = align_to(track.t_ns.astype(np.float64),
                        extras.satellites.t_ns.astype(np.float64),
                        extras.satellites.v, mode='nearest')
        ax.plot(track.t_s, sats, lw=1.0, color='tab:green', label='satellites')
        ax.set_ylabel('Satellites', color='tab:green')
        ax.tick_params(axis='y', labelcolor='tab:green')
        drew = True
    if extras.eph_m is not None and extras.eph_m.t_ns.size and np.any(np.isfinite(extras.eph_m.v)):
        eph = align_to(track.t_ns.astype(np.float64),
                       extras.eph_m.t_ns.astype(np.float64),
                       extras.eph_m.v, mode='linear')
        ax2 = ax.twinx()
        ax2.plot(track.t_s, eph, lw=1.0, color='tab:orange', label='EPH (m)')
        ax2.set_ylabel('EPH (m)', color='tab:orange')
        ax2.tick_params(axis='y', labelcolor='tab:orange')
        drew = True
    if not drew:
        ax.text(0.5, 0.5, 'no satellites / GPSRAW topics',
                ha='center', va='center', transform=ax.transAxes)
    ax.set_xlabel('Time (s)')
    ax.set_title('GPS quality vs time')
    ax.grid(alpha=0.3)


# ---- IO ---------------------------------------------------------------------


def write_path_csv(
    path: Path,
    track: GpsTrack,
    extras: GpsExtras,
    extra_columns: Optional[Dict[str, np.ndarray]] = None,
) -> None:
    t_target = track.t_ns.astype(np.float64)
    speed = (align_to(t_target, extras.speed.t_ns.astype(np.float64), extras.speed.v)
             if extras.speed is not None else np.full(t_target.shape, np.nan))
    vspeed = (align_to(t_target, extras.vspeed.t_ns.astype(np.float64), extras.vspeed.v)
              if extras.vspeed is not None else np.full(t_target.shape, np.nan))
    sats = (align_to(t_target, extras.satellites.t_ns.astype(np.float64),
                     extras.satellites.v, mode='nearest')
            if extras.satellites is not None else np.full(t_target.shape, np.nan))
    fix_type = (align_to(t_target, extras.fix_type.t_ns.astype(np.float64),
                         extras.fix_type.v, mode='nearest')
                if extras.fix_type is not None else np.full(t_target.shape, np.nan))
    eph = (align_to(t_target, extras.eph_m.t_ns.astype(np.float64), extras.eph_m.v)
           if extras.eph_m is not None else np.full(t_target.shape, np.nan))

    t_unix = track.t_ns.astype(np.float64) * 1e-9
    t_rel = track.t_s
    cov_xy = track.cov_xy
    extra_columns = extra_columns or {}

    with path.open('w', newline='') as f:
        w = csv.writer(f)
        header = [
            't_unix', 't_rel_s', 'lat', 'lon', 'alt_m',
            'speed_mps', 'vspeed_mps',
            'satellites', 'fix_type', 'eph_m',
            'nav_status', 'cov_xy_m',
        ] + list(extra_columns.keys())
        w.writerow(header)
        for i in range(len(track)):
            row = [
                f'{t_unix[i]:.6f}', f'{t_rel[i]:.6f}',
                f'{track.lat[i]:.8f}', f'{track.lon[i]:.8f}', f'{track.alt[i]:.3f}',
                _fmt(speed[i]), _fmt(vspeed[i]),
                _fmt(sats[i]), _fmt(fix_type[i]), _fmt(eph[i]),
                int(track.nav_status[i]), _fmt(cov_xy[i]),
            ]
            for col_vals in extra_columns.values():
                row.append(_fmt(col_vals[i]))
            w.writerow(row)


def _fmt(x: float) -> str:
    if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
        return ''
    return f'{x:.4f}'


def write_report(path: Path, bag_path: Path, resolved: Dict[str, str],
                 track: GpsTrack, extras: GpsExtras) -> None:
    lines: List[str] = []
    lines.append(f'bag: {bag_path}')
    lines.append('topics resolved:')
    for k, v in resolved.items():
        lines.append(f'  {k}: {v}')
    lines.append('')
    if len(track) == 0:
        lines.append('No valid GPS fixes parsed.')
        path.write_text('\n'.join(lines) + '\n')
        return

    duration_s = float(track.t_s[-1] - track.t_s[0]) if len(track) > 1 else 0.0
    distance_m = haversine_path_length(track.lat, track.lon)
    lines.append(f'fixes: {len(track)}')
    lines.append(f'duration: {duration_s:.2f} s')
    lines.append(f'path length (haversine): {distance_m:.1f} m')
    lines.append(f'lat range: [{track.lat.min():.6f}, {track.lat.max():.6f}]')
    lines.append(f'lon range: [{track.lon.min():.6f}, {track.lon.max():.6f}]')
    lines.append(f'alt range: [{track.alt.min():.2f}, {track.alt.max():.2f}] m')
    lines.append(f'start: ({track.lat[0]:.7f}, {track.lon[0]:.7f}) @ alt {track.alt[0]:.1f} m')
    lines.append(f'end:   ({track.lat[-1]:.7f}, {track.lon[-1]:.7f}) @ alt {track.alt[-1]:.1f} m')
    lines.append('')
    if extras.speed is not None and extras.speed.v.size:
        sp = extras.speed.v
        lines.append(
            f'gps_vel speed (m/s): mean={sp.mean():.2f}  '
            f'p50={np.median(sp):.2f}  p95={np.percentile(sp, 95):.2f}  '
            f'max={sp.max():.2f}'
        )
    if extras.satellites is not None and extras.satellites.v.size:
        s = extras.satellites.v
        lines.append(
            f'satellites: mean={s.mean():.1f}  min={int(s.min())}  max={int(s.max())}'
        )
    if extras.eph_m is not None and np.any(np.isfinite(extras.eph_m.v)):
        e = extras.eph_m.v[np.isfinite(extras.eph_m.v)]
        lines.append(
            f'EPH (m): mean={e.mean():.2f}  p50={np.median(e):.2f}  '
            f'p95={np.percentile(e, 95):.2f}'
        )
    if extras.fix_type is not None and extras.fix_type.v.size:
        ft = extras.fix_type.v.astype(int)
        unique, counts = np.unique(ft, return_counts=True)
        breakdown = ', '.join(
            f'{_FIX_TYPE_LABELS.get(int(u), str(int(u)))}={int(c)}'
            for u, c in zip(unique, counts)
        )
        lines.append(f'fix_type breakdown: {breakdown}')
    if track.cov_xy.size and np.any(np.isfinite(track.cov_xy)):
        c = track.cov_xy[np.isfinite(track.cov_xy)]
        lines.append(
            f'NavSatFix sigma_xy (m): mean={c.mean():.2f}  '
            f'p50={np.median(c):.2f}  p95={np.percentile(c, 95):.2f}'
        )

    path.write_text('\n'.join(lines) + '\n')


# ---- pipeline ---------------------------------------------------------------


def run(args: argparse.Namespace) -> None:
    bag_path = args.bag.expanduser()
    if not bag_path.exists():
        print(f'Path does not exist: {bag_path}', file=sys.stderr)
        sys.exit(1)

    requested = {
        'gps_fix':  args.gps_topic,
        'gps_vel':  args.vel_topic,
        'gps_sat':  args.sat_topic,
        'gpsraw':   args.gpsraw_topic,
    }
    expected = {
        'gps_fix':  'sensor_msgs/msg/NavSatFix',
        'gps_vel':  'geometry_msgs/msg/TwistStamped',
        'gps_sat':  'std_msgs/msg/UInt32',
        'gpsraw':   'mavros_msgs/msg/GPSRAW',
    }

    records, resolved = read_topics(bag_path, requested, expected)
    if 'gps_fix' not in resolved:
        print('Bag has no NavSatFix topic; cannot plot path.', file=sys.stderr)
        sys.exit(2)

    track = parse_navsatfix(records.get('gps_fix', []), args.stride)
    if len(track) == 0:
        print('NavSatFix topic present but no usable fixes (all-zero / NaN).', file=sys.stderr)
        sys.exit(3)

    extras = GpsExtras()
    if 'gps_vel' in resolved:
        speed_ts, vspeed_ts = parse_twist_speed(records['gps_vel'])
        extras.speed = speed_ts
        extras.vspeed = vspeed_ts
    if 'gps_sat' in resolved:
        extras.satellites = parse_uint32(records['gps_sat'], label='satellites')
    if 'gpsraw' in resolved:
        ft, eph = parse_gpsraw(records['gpsraw'])
        extras.fix_type = ft
        extras.eph_m = eph

    print(f'Parsed {len(track)} GPS fixes (stride={args.stride}).')

    out_stem = bag_path.name if bag_path.is_dir() else bag_path.parent.name
    out_dir = (args.output_dir.expanduser() if args.output_dir is not None
               else bag_path.parent / f'{out_stem}_geoplot')
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'Writing geoplot to: {out_dir}')

    color_vals, color_label = _select_color_values(track, extras, args.color_by)
    speed_aligned = (
        align_to(track.t_ns.astype(np.float64),
                 extras.speed.t_ns.astype(np.float64), extras.speed.v)
        if extras.speed is not None else np.full(track.t_ns.shape, np.nan)
    )

    dpi = float(args.dpi)
    lat0 = float(np.median(track.lat))
    lon0 = float(np.median(track.lon))

    fig, ax = plt.subplots(figsize=(10, 9), dpi=dpi)
    plot_path_local(ax, track, color_vals, color_label, args.cmap, lat0=lat0, lon0=lon0)
    fig.tight_layout()
    fig.savefig(out_dir / 'path_local.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 9), dpi=dpi)
    plot_path_geo(ax, track, color_vals, color_label, args.cmap)
    fig.tight_layout()
    fig.savefig(out_dir / 'path_geo.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=dpi)
    plot_altitude(ax, track)
    fig.tight_layout()
    fig.savefig(out_dir / 'altitude.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=dpi)
    plot_speed(ax, track, speed_aligned)
    fig.tight_layout()
    fig.savefig(out_dir / 'speed.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=dpi)
    plot_satellites(ax, track, extras)
    fig.tight_layout()
    fig.savefig(out_dir / 'satellites.png')
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(18, 13), dpi=dpi)
    plot_path_local(axes[0, 0], track, color_vals, color_label, args.cmap,
                    lat0=lat0, lon0=lon0)
    plot_altitude(axes[0, 1], track)
    plot_speed(axes[1, 0], track, speed_aligned)
    plot_satellites(axes[1, 1], track, extras)
    fig.suptitle(
        f'{bag_path.name} — {len(track)} GPS fixes, '
        f'{haversine_path_length(track.lat, track.lon):.0f} m path, '
        f'{(track.t_s[-1] - track.t_s[0]):.0f} s',
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / 'summary.png')
    plt.close(fig)

    extra_csv: Dict[str, np.ndarray] = {}
    overlay_files: List[str] = []

    if 'laser' in args.overlay:
        overlay_files += _run_laser_overlay(
            bag_path, args, track, lat0, lon0, dpi, out_dir, extra_csv,
        )

    if 'spectra-clusters' in args.overlay:
        overlay_files += _run_spectra_clusters_overlay(
            bag_path, args, track, lat0, lon0, dpi, out_dir, extra_csv,
        )

    write_path_csv(out_dir / 'path.csv', track, extras, extra_csv)
    write_report(out_dir / 'report.txt', bag_path, resolved, track, extras)

    base_msg = f'Wrote {out_dir / "summary.png"} (+ 5 panels + path.csv + report.txt)'
    if overlay_files:
        base_msg += f'; {len(overlay_files)} overlay figures: ' + ', '.join(overlay_files)
    print(base_msg + '.')


# ---- overlay implementations ------------------------------------------------


def _run_laser_overlay(
    bag_path: Path,
    args: argparse.Namespace,
    track: GpsTrack,
    lat0: float,
    lon0: float,
    dpi: float,
    out_dir: Path,
    extra_csv: Dict[str, np.ndarray],
) -> List[str]:
    print(f'[overlay:laser] reading {args.laser_topic}')
    records, resolved = read_topics(
        bag_path,
        {'laser': args.laser_topic},
        {'laser': 'std_msgs/msg/Float64'},
    )
    if 'laser' not in resolved:
        print('  laser topic not found; skipping overlay.', file=sys.stderr)
        return []
    laser_ts = parse_scalar(records['laser'], label='laser_distance', units='m')
    if laser_ts.t_ns.size == 0:
        print('  laser topic has 0 messages; skipping overlay.', file=sys.stderr)
        return []
    laser_aligned = align_to(
        track.t_ns.astype(np.float64),
        laser_ts.t_ns.astype(np.float64),
        laser_ts.v,
        mode='linear',
    )
    finite = np.isfinite(laser_aligned)
    if not finite.any():
        print('  laser samples did not overlap GPS times; skipping overlay.',
              file=sys.stderr)
        return []

    fig, ax = plt.subplots(figsize=(10, 9), dpi=dpi)
    plot_path_local(
        ax, track, laser_aligned, 'Laser distance (m)', 'viridis',
        lat0=lat0, lon0=lon0,
        title='Path colored by laser distance',
    )
    fig.tight_layout()
    fig.savefig(out_dir / 'path_laser.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=dpi)
    laser_t_s = (laser_ts.t_ns - track.t_ns[0]).astype(np.float64) * 1e-9
    ax.plot(laser_t_s, laser_ts.v, lw=0.6, color='tab:cyan', alpha=0.6,
            label='raw')
    ax.plot(track.t_s, laser_aligned, lw=1.0, color='tab:blue',
            label='aligned to fixes')
    ax.set_xlabel('Time (s, GPS-fix relative)')
    ax.set_ylabel('Laser distance (m)')
    ax.set_title('Laser ranger vs time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_dir / 'laser.png')
    plt.close(fig)

    extra_csv['laser_m'] = laser_aligned
    finite_v = laser_aligned[finite]
    print(
        f'  laser: {laser_ts.t_ns.size} samples;'
        f' aligned mean={finite_v.mean():.2f} m,'
        f' p50={np.median(finite_v):.2f},'
        f' p95={np.percentile(finite_v, 95):.2f},'
        f' max={finite_v.max():.2f}'
    )
    return ['path_laser.png', 'laser.png']


def _run_spectra_clusters_overlay(
    bag_path: Path,
    args: argparse.Namespace,
    track: GpsTrack,
    lat0: float,
    lon0: float,
    dpi: float,
    out_dir: Path,
    extra_csv: Dict[str, np.ndarray],
) -> List[str]:
    print(f'[overlay:spectra-clusters] reading {args.spectra_topic}')
    try:
        times_s_spec, wl, X, resolved_spec, stats = load_spectra_from_bag(
            bag_path=bag_path,
            topic=args.spectra_topic,
            time_stride=args.spectra_stride,
        )
    except Exception as exc:
        print(f'  load_spectra_from_bag failed: {exc}', file=sys.stderr)
        return []
    spec_t_ns = stats['times_ns']
    print(
        f'  spectra: {X.shape[0]} samples × {X.shape[1]} channels '
        f'from {resolved_spec}'
    )

    compute_clusters_and_nmf = _import_cluster_pipeline()
    print('  running clustering pipeline (preproc → PCA → UMAP/HDBSCAN → NMF) ...')
    res = compute_clusters_and_nmf(
        X, wl,
        mask_lo=args.spectra_mask_lo,
        mask_hi=args.spectra_mask_hi,
        nmf_components=args.spectra_nmf_components,
        hdbscan_min_cluster=args.spectra_min_cluster,
        random_seed=args.random_seed,
    )
    n_clusters = int(np.unique(res.labels[res.labels >= 0]).size)
    n_noise = int((res.labels == -1).sum())
    print(
        f'  clusters={n_clusters}  noise={n_noise}/{res.labels.size}  '
        f'NMF K={res.nmf_endmembers.shape[0]}'
    )

    target_t = track.t_ns.astype(np.float64)
    src_t = spec_t_ns.astype(np.float64)
    cluster_aligned = align_to(
        target_t, src_t, res.labels.astype(np.float64), mode='nearest',
    )

    fig, ax = plt.subplots(figsize=(11, 9), dpi=dpi)
    plot_path_clusters(
        ax, track, cluster_aligned, lat0, lon0,
        title=f'Path colored by spectra cluster ({n_clusters} clusters, NaN=no spectra)',
    )
    fig.tight_layout()
    fig.savefig(out_dir / 'path_clusters.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4), dpi=dpi)
    cmap = plt.get_cmap('tab20')
    for lab in np.unique(res.labels):
        m = res.labels == lab
        if not m.any():
            continue
        c = '#999' if lab == -1 else cmap(int(lab) % 20)
        ax.scatter(times_s_spec[m], np.full(int(m.sum()), int(lab)),
                   c=[c], s=8, alpha=0.85,
                   label=('noise' if lab == -1 else f'c{int(lab)}'))
    ax.set_xlabel('Time (s, spectrometer-relative)')
    ax.set_ylabel('Cluster id')
    ax.set_title('Spectra cluster id vs time')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=min(8, max(2, n_clusters + 1)))
    fig.tight_layout()
    fig.savefig(out_dir / 'clusters_timeline.png')
    plt.close(fig)

    extra_csv['spectra_cluster'] = cluster_aligned

    abund = res.nmf_abundances
    abund_share = abund / np.clip(abund.sum(axis=1, keepdims=True), 1e-12, None)
    written: List[str] = ['path_clusters.png', 'clusters_timeline.png']
    for i in range(abund_share.shape[1]):
        em_aligned = align_to(target_t, src_t, abund_share[:, i], mode='linear')
        fig, ax = plt.subplots(figsize=(10, 9), dpi=dpi)
        peak_nm = float(res.wl_masked[int(np.argmax(res.nmf_endmembers[i]))])
        plot_path_local(
            ax, track, em_aligned,
            f'NMF EM{i} share',
            'viridis',
            lat0=lat0, lon0=lon0,
            title=f'Path colored by NMF endmember EM{i} (peak ≈ {peak_nm:.1f} nm)',
        )
        fig.tight_layout()
        fig.savefig(out_dir / f'path_nmf_em{i}.png')
        plt.close(fig)
        extra_csv[f'nmf_em{i}'] = em_aligned
        written.append(f'path_nmf_em{i}.png')

    return written


def main() -> None:
    p = argparse.ArgumentParser(
        description='Plot trike GPS path from a rosbag (with speed / sat / EPH overlays).',
    )
    p.add_argument('--bag', '-b', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, default=None,
                   help='Directory for figures, CSV, and report (default: <bag>_geoplot next to bag).')
    p.add_argument('--gps-topic', default='/mavros/global_position/raw/fix',
                   help='NavSatFix topic for the path (default: %(default)s).')
    p.add_argument('--vel-topic', default='/mavros/global_position/raw/gps_vel',
                   help='TwistStamped ground velocity topic (default: %(default)s; pass "" to skip).')
    p.add_argument('--sat-topic', default='/mavros/global_position/raw/satellites',
                   help='UInt32 satellite count topic (default: %(default)s; pass "" to skip).')
    p.add_argument('--gpsraw-topic', default='/mavros/gpsstatus/gps1/raw',
                   help='mavros_msgs/GPSRAW topic for fix_type + EPH (default: %(default)s; pass "" to skip).')
    p.add_argument('--color-by', default='speed',
                   choices=['speed', 'time', 'altitude', 'satellites', 'fix_type'],
                   help='Per-sample color value on the path plots.')
    p.add_argument('--stride', type=int, default=1,
                   help='Decimate GPS fixes (keep every Nth).')
    p.add_argument('--cmap', default='viridis')
    p.add_argument('--dpi', type=float, default=140.0)
    p.add_argument('--random-seed', type=int, default=0)

    # ---- overlays ---------------------------------------------------------
    p.add_argument(
        '--overlay',
        action='append',
        default=[],
        choices=['laser', 'spectra-clusters'],
        help=(
            'Add an overlay layer (repeatable). "laser" reads /laser_distance, '
            '"spectra-clusters" runs the unsupervised cluster pipeline on '
            '/spectrometer and colors the path by cluster id + per-NMF endmember.'
        ),
    )
    p.add_argument(
        '--laser-topic',
        default='/laser_distance',
        help='Float64 laser distance topic (default: %(default)s).',
    )
    p.add_argument(
        '--spectra-topic',
        default='/spectrometer',
        help='Float64MultiArray spectrometer topic (default: %(default)s).',
    )
    p.add_argument(
        '--spectra-stride',
        type=int,
        default=1,
        help='Decimate spectra (keep every Nth) before clustering.',
    )
    p.add_argument(
        '--spectra-mask-lo',
        type=float,
        default=350.0,
        help='Drop wavelengths below this (nm) before clustering. '
             'Default 350 trims the noisy UV edge typical of OceanOptics.',
    )
    p.add_argument(
        '--spectra-mask-hi',
        type=float,
        default=850.0,
        help='Drop wavelengths above this (nm) before clustering.',
    )
    p.add_argument(
        '--spectra-nmf-components',
        type=int,
        default=5,
        help='Number of NMF endmembers (default %(default)d).',
    )
    p.add_argument(
        '--spectra-min-cluster',
        type=int,
        default=0,
        help='HDBSCAN min_cluster_size on spectra (0 = auto = max(20, N/100)).',
    )

    args = p.parse_args()

    if args.stride < 1:
        print('--stride must be >= 1', file=sys.stderr)
        sys.exit(1)

    for attr in ('vel_topic', 'sat_topic', 'gpsraw_topic'):
        val = getattr(args, attr)
        if val == '':
            setattr(args, attr, None)

    run(args)


if __name__ == '__main__':
    main()
