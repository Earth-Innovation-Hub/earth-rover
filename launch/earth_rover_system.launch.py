#!/usr/bin/env python3
"""Sequenced Earth Rover system launch (trike profile).

Default startup order:
  1. MAVROS / PX4 bridge
  2. radio_vio: RTL-SDR ADS-B + landmark VO stack
  3. RTL-SDR (standalone spectrum analyzer)  -- OFF by default; do not enable
     simultaneously with radio_vio if both share the same dongle
  4. Laser ranger (USB serial)
  5. Grasshopper stereo cameras
  6. Spectrometer publisher
  7. rqt GUI loaded with the earth_rover perspective

Each stage is delayed enough to let the previous process initialize. The delays
are launch arguments so field tuning does not require editing this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_perspective_path(value: str) -> str:
    """Find the rqt perspective file by trying a few sensible locations."""
    value = os.path.expanduser(os.path.expandvars(value.strip()))
    if not value:
        value = 'earth_rover.perspective'
    if os.path.isabs(value) and os.path.isfile(value):
        return value
    candidates = []
    if os.sep in value:
        candidates.append(os.path.abspath(value))
    else:
        try:
            share = get_package_share_directory('deepgis_vehicles')
            candidates.append(os.path.join(share, 'config', value))
            candidates.append(os.path.join(share, value))
        except Exception:
            pass
        # Fall back to the source tree so an un-installed checkout still works.
        candidates.append(os.path.join(
            os.environ.get('EARTH_ROVER_HOME', '/home/jdas/earth-rover'),
            'config', value,
        ))
        candidates.append(os.path.join(
            os.environ.get('EARTH_ROVER_HOME', '/home/jdas/earth-rover'),
            value,
        ))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return ''


def _bool_arg(context, name: str) -> bool:
    return str(LaunchConfiguration(name).perform(context)).lower() in ('1', 'true', 'yes', 'on')


def _float_arg(context, name: str) -> float:
    return float(LaunchConfiguration(name).perform(context))


def _launch_file(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), 'launch', *parts)


def _include(package: str, launch_file: str, launch_arguments=None):
    path = _launch_file(package, launch_file)
    source = PythonLaunchDescriptionSource(path) if path.endswith('.launch.py') else AnyLaunchDescriptionSource(path)
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
    )


def _optional_stage(context, package: str, launch_file: str, launch_arguments=None):
    try:
        return [_include(package, launch_file, launch_arguments)]
    except Exception as exc:
        return [LogInfo(msg=f'[earth_rover_system] Skipping {package}/{launch_file}: {exc}')]


def launch_setup(context, *args, **kwargs):
    actions = []

    # Stage 1: MAVROS. Required for global/local position consumers downstream.
    if _bool_arg(context, 'enable_mavros'):
        actions.extend([
            LogInfo(msg='[earth_rover_system] Stage 1: starting MAVROS'),
            _include('mavros', 'px4.launch', {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'namespace': LaunchConfiguration('mavros_namespace'),
            }),
        ])

    # Stage 2: RTL-SDR / ADS-B / radio_vio.
    if _bool_arg(context, 'enable_radio_vio'):
        actions.append(TimerAction(
            period=_float_arg(context, 'radio_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 2: starting radio_vio'),
                *_optional_stage(context, 'radio_vio', 'radio_stack.launch.py', {
                    'preset': LaunchConfiguration('radio_preset'),
                    'decoder_mode': LaunchConfiguration('decoder_mode'),
                    'gain': LaunchConfiguration('rtl_gain'),
                    'device_index': LaunchConfiguration('rtl_device_index'),
                }),
            ],
        ))

    # Stage 3: standalone RTL-SDR spectrum analyzer. Off by default so it does
    # not contend with radio_vio for the same dongle. Use a different
    # rtl_sdr_device_index here if you want both up at once on a multi-dongle rig.
    if _bool_arg(context, 'enable_rtl_sdr'):
        actions.append(TimerAction(
            period=_float_arg(context, 'rtl_sdr_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 3: starting standalone RTL-SDR'),
                *_optional_stage(context, 'radio_vio', 'rtl_sdr.launch.py', {
                    'frequency': LaunchConfiguration('rtl_sdr_frequency'),
                    'sample_rate': LaunchConfiguration('rtl_sdr_sample_rate'),
                    'gain': LaunchConfiguration('rtl_sdr_gain'),
                    'device_index': LaunchConfiguration('rtl_sdr_device_index'),
                    'bias_tee': LaunchConfiguration('rtl_sdr_bias_tee'),
                    'enable_spectrum_analyzer': LaunchConfiguration('rtl_sdr_enable_spectrum_analyzer'),
                    'enable_visualizer': LaunchConfiguration('rtl_sdr_enable_visualizer'),
                    'publish_raw_iq': LaunchConfiguration('rtl_sdr_publish_raw_iq'),
                }),
            ],
        ))

    # Stage 4: laser ranger (USB serial). Default device matches the trike build
    # observed in recent history; override via laser_serial_device:=/dev/ttyUSBn.
    if _bool_arg(context, 'enable_laser'):
        actions.append(TimerAction(
            period=_float_arg(context, 'laser_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 4: starting laser ranger'),
                *_optional_stage(context, 'laser_ranger', 'laser_ranger.launch.py', {
                    'serial_device': LaunchConfiguration('laser_serial_device'),
                    'baud_rate': LaunchConfiguration('laser_baud_rate'),
                    'topic_distance': LaunchConfiguration('laser_topic_distance'),
                }),
            ],
        ))

    # Stage 5: cameras. Grasshopper left+right are launched as independent nodes.
    camera_actions = []
    if _bool_arg(context, 'enable_grasshopper'):
        shared_param_file = LaunchConfiguration('grasshopper_parameter_file').perform(context)
        left_param_file = LaunchConfiguration('grasshopper_left_parameter_file').perform(context) or shared_param_file
        right_param_file = LaunchConfiguration('grasshopper_right_parameter_file').perform(context) or shared_param_file
        shared_rate = LaunchConfiguration('grasshopper_frame_rate').perform(context)
        left_rate = LaunchConfiguration('grasshopper_left_frame_rate').perform(context) or shared_rate
        right_rate = LaunchConfiguration('grasshopper_right_frame_rate').perform(context) or shared_rate
        camera_actions.extend(_optional_stage(context, 'spinnaker_camera_driver', 'grasshopper_left.launch.py', {
            'serial': LaunchConfiguration('grasshopper_left_serial'),
            'parameter_file': left_param_file,
            'camera_info_url': LaunchConfiguration('grasshopper_left_camera_info_url'),
            'frame_rate': left_rate,
        }))
        camera_actions.extend(_optional_stage(context, 'spinnaker_camera_driver', 'grasshopper_right.launch.py', {
            'serial': LaunchConfiguration('grasshopper_right_serial'),
            'parameter_file': right_param_file,
            'camera_info_url': LaunchConfiguration('grasshopper_right_camera_info_url'),
            'frame_rate': right_rate,
        }))
    if _bool_arg(context, 'enable_realsense'):
        camera_actions.extend(_optional_stage(context, 'realsense2_camera', 'rs_launch.py'))

    if camera_actions:
        actions.append(TimerAction(
            period=_float_arg(context, 'camera_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 5: starting cameras'),
                *camera_actions,
            ],
        ))

    # Stage 6: spectrometer.
    if _bool_arg(context, 'enable_spectrometer'):
        actions.append(TimerAction(
            period=_float_arg(context, 'spectrometer_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 6: starting spectrometer'),
                *_optional_stage(context, 'spectrometery_ros2', 'spectrometer_data_publisher.launch.py', {
                    'plot_image': LaunchConfiguration('spectrometer_plot_image'),
                    'integration_time_micros': LaunchConfiguration('spectrometer_integration_time_micros'),
                    'publish_period_sec': LaunchConfiguration('spectrometer_publish_period_sec'),
                }),
            ],
        ))

    # Stage 7: rqt GUI on the host display, loaded with the earth_rover perspective.
    if _bool_arg(context, 'rqt_gui'):
        perspective_path = _resolve_perspective_path(
            LaunchConfiguration('rqt_perspective').perform(context))
        if not os.environ.get('DISPLAY'):
            actions.append(TimerAction(
                period=_float_arg(context, 'rqt_delay_sec'),
                actions=[LogInfo(msg='[earth_rover_system] Stage 7: skipping rqt -- DISPLAY is not set.')],
            ))
        else:
            cmd = ['rqt']
            if perspective_path:
                cmd += ['--perspective-file', perspective_path]
                msg = f'[earth_rover_system] Stage 7: starting rqt with perspective {perspective_path}'
            else:
                msg = '[earth_rover_system] Stage 7: starting rqt (perspective not found, using default)'
            actions.append(TimerAction(
                period=_float_arg(context, 'rqt_delay_sec'),
                actions=[
                    LogInfo(msg=msg),
                    ExecuteProcess(
                        cmd=cmd,
                        output='screen',
                        emulate_tty=True,
                        log_cmd=True,
                    ),
                ],
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_mavros', default_value='true'),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600',
        ),
        DeclareLaunchArgument('gcs_url', default_value='udp://@192.168.0.6:14550'),
        DeclareLaunchArgument('mavros_namespace', default_value='/mavros'),

        DeclareLaunchArgument('enable_radio_vio', default_value='true'),
        DeclareLaunchArgument('radio_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('radio_preset', default_value='full'),
        DeclareLaunchArgument('decoder_mode', default_value='dump1090'),
        DeclareLaunchArgument('rtl_gain', default_value='-1.0'),
        DeclareLaunchArgument('rtl_device_index', default_value='0'),
        # MAVROS publishes under /mavros. radio_vio nodes (especially
        # adsb_aircraft_state_vectors_node, landmark_vo_plot_*) read this same
        # arg and concatenate it into absolute topic paths, so it MUST start
        # with a leading slash -- a bare 'mavros' would resolve under each
        # consumer node's own namespace and silently miss every MAVROS topic.

        DeclareLaunchArgument('enable_rtl_sdr', default_value='false',
                              description='Standalone RTL-SDR + spectrum analyzer (do not run alongside radio_vio on the same dongle).'),
        DeclareLaunchArgument('rtl_sdr_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('rtl_sdr_frequency', default_value='433.0e6'),
        DeclareLaunchArgument('rtl_sdr_sample_rate', default_value='2.4e6'),
        DeclareLaunchArgument('rtl_sdr_gain', default_value='-1.0'),
        DeclareLaunchArgument('rtl_sdr_device_index', default_value='1',
                              description='Set to 0 if no other dongle is in use; default 1 keeps it off radio_vio dongle 0.'),
        DeclareLaunchArgument('rtl_sdr_bias_tee', default_value='false'),
        DeclareLaunchArgument('rtl_sdr_enable_spectrum_analyzer', default_value='true'),
        DeclareLaunchArgument('rtl_sdr_enable_visualizer', default_value='false'),
        DeclareLaunchArgument('rtl_sdr_publish_raw_iq', default_value='true'),

        DeclareLaunchArgument('enable_laser', default_value='true'),
        DeclareLaunchArgument('laser_delay_sec', default_value='12.0'),
        DeclareLaunchArgument(
            'laser_serial_device',
            default_value='/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO01SSV5-if00-port0',
            description=(
                'Serial port for the laser ranger. Use a /dev/serial/by-id/... '
                'path so the device survives unplug/replug and competes-for-tty '
                'reordering with the Pixhawk FTDI on /dev/ttyUSB*.'
            ),
        ),
        DeclareLaunchArgument('laser_baud_rate', default_value='115200'),
        DeclareLaunchArgument('laser_topic_distance', default_value='laser_distance'),

        DeclareLaunchArgument('camera_delay_sec', default_value='16.0'),
        DeclareLaunchArgument('enable_grasshopper', default_value='true'),
        DeclareLaunchArgument('grasshopper_left_serial', default_value='22312692'),
        DeclareLaunchArgument('grasshopper_right_serial', default_value='22312674'),
        DeclareLaunchArgument('grasshopper_parameter_file', default_value='grasshopper.yaml'),
        DeclareLaunchArgument('grasshopper_left_parameter_file', default_value=''),
        DeclareLaunchArgument('grasshopper_right_parameter_file', default_value=''),
        DeclareLaunchArgument('grasshopper_left_camera_info_url', default_value=''),
        DeclareLaunchArgument('grasshopper_right_camera_info_url', default_value=''),
        DeclareLaunchArgument('grasshopper_frame_rate', default_value='15.0',
                              description='Free-run frame rate (Hz) applied to both Grasshopper3 cameras unless per-side override is set.'),
        DeclareLaunchArgument('grasshopper_left_frame_rate', default_value='',
                              description='Per-side override for the left camera; empty -> use grasshopper_frame_rate.'),
        DeclareLaunchArgument('grasshopper_right_frame_rate', default_value='',
                              description='Per-side override for the right camera; empty -> use grasshopper_frame_rate.'),
        DeclareLaunchArgument('enable_realsense', default_value='false'),

        DeclareLaunchArgument('enable_spectrometer', default_value='true'),
        DeclareLaunchArgument('spectrometer_delay_sec', default_value='24.0'),
        DeclareLaunchArgument('spectrometer_plot_image', default_value='true'),
        DeclareLaunchArgument('spectrometer_integration_time_micros', default_value='500000'),
        DeclareLaunchArgument('spectrometer_publish_period_sec', default_value='0.1'),

        DeclareLaunchArgument('rqt_gui', default_value='true',
                              description='Launch rqt on the host display with the earth_rover perspective.'),
        DeclareLaunchArgument('rqt_delay_sec', default_value='28.0',
                              description='Delay before launching rqt so cameras/MAVROS topics are subscribable.'),
        DeclareLaunchArgument('rqt_perspective', default_value='earth_rover.perspective',
                              description='rqt perspective file (basename, relative path, or absolute path). Searched in package share/config, share/, then $EARTH_ROVER_HOME.'),

        OpaqueFunction(function=launch_setup),
    ])
