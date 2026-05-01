#!/usr/bin/env python3
"""Earth Rover rosbag2 recorder.

Records the project's standard topic set, grouped into "classes" that can be
toggled on/off from the command line. Topics that are not currently published
are skipped (per-class), so the bag still records whatever IS available instead
of failing the whole launch. If you want missing topics to be recorded as soon
as they appear, set ``wait_for_topics:=true`` and they will be passed to
``ros2 bag record`` regardless of current presence.

Examples
--------
Default (all classes except heavy raw stereo, mcap, no compression)::

    ros2 launch deepgis_vehicles record_bag.launch.py

Quick MAVROS-only telemetry capture::

    ros2 launch deepgis_vehicles record_bag.launch.py \
        record_stereo_compressed:=false \
        record_stereo_raw:=false \
        record_spectrometer:=false \
        record_laser:=false

Full sensor capture with raw stereo + zstd compression and 2 GiB splits::

    ros2 launch deepgis_vehicles record_bag.launch.py \
        record_stereo_raw:=true \
        compression_mode:=file compression_format:=zstd \
        max_bag_size:=2147483648

Pre-subscribe to topics that aren't up yet (e.g. cameras start later)::

    ros2 launch deepgis_vehicles record_bag.launch.py wait_for_topics:=true

Add ad-hoc topics::

    ros2 launch deepgis_vehicles record_bag.launch.py extra_topics:=/foo,/bar
"""

import datetime as _dt
import os
import time

import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# ---------------------------------------------------------------------------
# Topic class registry. Each entry is (class_name, [topic, ...]).
# Add new classes here; flip them on/off via record_<class>:=true|false.
# ---------------------------------------------------------------------------
TOPIC_CLASSES = {
    'mavros_state': [
        '/mavros/state',
        '/mavros/extended_state',
        '/mavros/estimator_status',
        '/mavros/home_position/home',
        '/mavros/battery',
        '/mavros/onboard_computer/status',
        '/mavros/esc_status/status',
        '/mavros/actuator_control',
        '/mavros/gimbal_control/manager/status',
    ],
    'mavros_imu': [
        '/mavros/imu/data',
        '/mavros/imu/mag',
        '/mavros/imu/diff_pressure',
        '/mavros/imu/static_pressure',
        '/mavros/imu/temperature_baro',
        '/mavros/imu/temperature_imu',
    ],
    'mavros_local_position': [
        '/mavros/local_position/accel',
        '/mavros/local_position/odom',
        '/mavros/local_position/pose',
        '/mavros/local_position/pose_cov',
        '/mavros/local_position/velocity_body',
        '/mavros/local_position/velocity_body_cov',
        '/mavros/local_position/velocity_local',
        '/mavros/odometry/out',
    ],
    'mavros_global_position': [
        '/mavros/altitude',
        '/mavros/global_position/compass_hdg',
        '/mavros/global_position/global',
        '/mavros/global_position/gp_lp_offset',
        '/mavros/global_position/gp_origin',
        '/mavros/global_position/local',
        '/mavros/global_position/raw/fix',
        '/mavros/global_position/raw/gps_vel',
        '/mavros/global_position/raw/satellites',
        '/mavros/global_position/rel_alt',
        '/mavros/global_position/set_gp_origin',
    ],
    'mavros_gps_status': [
        '/mavros/gpsstatus/gps1/raw',
        '/mavros/gpsstatus/gps1/rtk',
        '/mavros/gpsstatus/gps2/raw',
        '/mavros/gpsstatus/gps2/rtk',
    ],
    'stereo_raw': [
        '/stereo/left/image_raw',
        '/stereo/right/image_raw',
    ],
    'stereo_compressed': [
        '/stereo/left/image_raw/compressed',
        '/stereo/right/image_raw/compressed',
    ],
    'stereo_camera_info': [
        '/stereo/left/camera_info',
        '/stereo/right/camera_info',
    ],
    'spectrometer': [
        '/spectrometer',
        '/spectrometer_plot',
    ],
    'laser': [
        '/laser_distance',
    ],
    'adsb': [
        '/adsb/rtl_adsb_decoder_node/aircraft_list',
        '/adsb/rtl_adsb_decoder_node/estimated_position',
    ],
    'tf': [
        '/tf',
        '/tf_static',
    ],
    'diagnostics': [
        '/diagnostics',
    ],
}

# Defaults per class. stereo_raw is OFF by default since it dominates bandwidth.
CLASS_DEFAULTS = {name: 'true' for name in TOPIC_CLASSES}
CLASS_DEFAULTS['stereo_raw'] = 'false'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _bool_arg(context, name: str) -> bool:
    return str(LaunchConfiguration(name).perform(context)).lower() in (
        '1', 'true', 'yes', 'on'
    )


def _str_arg(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _split_csv(value: str):
    return [x.strip() for x in value.split(',') if x.strip()]


def _discover_active_topics(discovery_timeout_sec: float) -> set:
    """Spin a throwaway rclpy node briefly and return the set of live topics."""
    own_init = False
    if not rclpy.ok():
        rclpy.init()
        own_init = True
    try:
        node = rclpy.create_node('earth_rover_record_bag_probe')
        deadline = time.monotonic() + max(0.0, discovery_timeout_sec)
        seen = set()
        # Spin briefly so the discovery cache fills up.
        while True:
            for name, _types in node.get_topic_names_and_types():
                seen.add(name)
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        return seen
    finally:
        if own_init:
            rclpy.shutdown()


def _resolve_output_dir(context) -> str:
    base = os.path.expanduser(os.path.expandvars(_str_arg(context, 'bag_dir')))
    name = _str_arg(context, 'bag_name')
    if not name:
        name = 'earth_rover_' + _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    if not base:
        base = os.path.expanduser('~/earth-rover-bags')
    return os.path.join(base, name)


# ---------------------------------------------------------------------------
# launch_setup
# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):
    actions = []

    wait_for_topics = _bool_arg(context, 'wait_for_topics')
    discovery_timeout = float(_str_arg(context, 'discovery_timeout_sec') or '2.0')
    extra_topics = _split_csv(_str_arg(context, 'extra_topics'))
    exclude_topics = set(_split_csv(_str_arg(context, 'exclude_topics')))

    if wait_for_topics:
        active = None  # signals "skip availability filter"
        actions.append(LogInfo(msg='[record_bag] wait_for_topics=true: subscribing to all requested topics regardless of current availability.'))
    else:
        active = _discover_active_topics(discovery_timeout)
        actions.append(LogInfo(msg=f'[record_bag] discovery scan found {len(active)} live topic(s).'))

    selected = []  # preserves order, no duplicates
    selected_set = set()

    def _add(topic):
        if topic in exclude_topics or topic in selected_set:
            return
        selected.append(topic)
        selected_set.add(topic)

    for class_name, class_topics in TOPIC_CLASSES.items():
        if not _bool_arg(context, f'record_{class_name}'):
            actions.append(LogInfo(msg=f'[record_bag] class {class_name}: disabled'))
            continue
        if active is None:
            kept, missing = list(class_topics), []
        else:
            kept = [t for t in class_topics if t in active]
            missing = [t for t in class_topics if t not in active]
        for t in kept:
            _add(t)
        if missing:
            actions.append(LogInfo(
                msg=f'[record_bag] class {class_name}: recording {len(kept)}/{len(class_topics)}; '
                    f'skipping not-yet-published: {", ".join(missing)}'))
        else:
            actions.append(LogInfo(
                msg=f'[record_bag] class {class_name}: recording {len(kept)}/{len(class_topics)}'))

    for t in extra_topics:
        if active is not None and t not in active:
            actions.append(LogInfo(msg=f'[record_bag] extra topic {t}: not currently published, skipping'))
            continue
        _add(t)

    if not selected:
        actions.append(LogInfo(msg='[record_bag] no topics selected after filtering — nothing to record. Exiting.'))
        return actions

    output_dir = _resolve_output_dir(context)
    os.makedirs(os.path.dirname(output_dir) or '.', exist_ok=True)
    storage = _str_arg(context, 'storage') or 'mcap'
    compression_mode = _str_arg(context, 'compression_mode') or 'none'
    compression_format = _str_arg(context, 'compression_format') or 'zstd'
    max_bag_size = _str_arg(context, 'max_bag_size')        # bytes, '' = no split
    max_bag_duration = _str_arg(context, 'max_bag_duration')  # seconds, '' = no split
    polling_interval = _str_arg(context, 'polling_interval_ms') or '100'
    try:
        polling_interval = str(int(float(polling_interval)))
    except ValueError as exc:
        raise ValueError(f'polling_interval_ms must be an integer (ms), got {polling_interval!r}') from exc
    extra_args = _split_csv(_str_arg(context, 'extra_record_args'))

    cmd = [
        'ros2', 'bag', 'record',
        '-o', output_dir,
        '-s', storage,
        '-p', polling_interval,
        '--include-hidden-topics',
    ]
    if compression_mode != 'none':
        cmd += ['--compression-mode', compression_mode,
                '--compression-format', compression_format]
    if max_bag_size:
        cmd += ['-b', max_bag_size]
    if max_bag_duration:
        cmd += ['-d', max_bag_duration]
    cmd += extra_args
    cmd += ['--topics'] + selected

    actions.append(LogInfo(msg=f'[record_bag] writing to {output_dir}'))
    actions.append(LogInfo(msg=f'[record_bag] recording {len(selected)} topic(s).'))
    actions.append(ExecuteProcess(
        cmd=cmd,
        output='screen',
        emulate_tty=True,
        log_cmd=True,
    ))
    return actions


def generate_launch_description():
    declares = [
        DeclareLaunchArgument('bag_dir', default_value='~/earth-rover-bags',
                              description='Parent directory for the bag (created if missing).'),
        DeclareLaunchArgument('bag_name', default_value='',
                              description='Bag folder name. Empty -> earth_rover_<UTC_timestamp>.'),
        DeclareLaunchArgument('storage', default_value='mcap',
                              description='rosbag2 storage plugin: mcap or sqlite3.'),
        DeclareLaunchArgument('compression_mode', default_value='none',
                              description='Compression mode: none|file|message.'),
        DeclareLaunchArgument('compression_format', default_value='zstd',
                              description='Compression format when compression_mode != none.'),
        DeclareLaunchArgument('max_bag_size', default_value='',
                              description='Split bag at this size in bytes (empty = no split).'),
        DeclareLaunchArgument('max_bag_duration', default_value='',
                              description='Split bag at this duration in seconds (empty = no split).'),
        DeclareLaunchArgument('polling_interval_ms', default_value='100',
                              description='ros2 bag record polling interval in milliseconds (integer).'),
        DeclareLaunchArgument('discovery_timeout_sec', default_value='2.0',
                              description='How long to scan for live topics before deciding what to record.'),
        DeclareLaunchArgument('wait_for_topics', default_value='false',
                              description='If true, subscribe to all requested topics even if not yet published.'),
        DeclareLaunchArgument('extra_topics', default_value='',
                              description='Comma-separated extra topics to record.'),
        DeclareLaunchArgument('exclude_topics', default_value='',
                              description='Comma-separated topics to drop from the final list.'),
        DeclareLaunchArgument('extra_record_args', default_value='',
                              description='Comma-separated extra args appended to ros2 bag record (advanced).'),
    ]
    for class_name in TOPIC_CLASSES:
        declares.append(DeclareLaunchArgument(
            f'record_{class_name}',
            default_value=CLASS_DEFAULTS[class_name],
            description=f'Enable/disable the {class_name} topic class.',
        ))
    declares.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(declares)
