#!/usr/bin/env python3
"""Sequenced Earth Rover system launch.

Default startup order:
  1. MAVROS / PX4 bridge
  2. RTL-SDR ADS-B / radio_vio stack
  3. Grasshopper stereo cameras
  4. Spectrometer publisher

Each stage is delayed enough to let the previous process initialize. The delays
are launch arguments so field tuning does not require editing this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _bool_arg(context, name: str) -> bool:
    return str(LaunchConfiguration(name).perform(context)).lower() in ('1', 'true', 'yes', 'on')


def _float_arg(context, name: str) -> float:
    return float(LaunchConfiguration(name).perform(context))


def _launch_file(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), 'launch', *parts)


def _include(package: str, launch_file: str, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file(package, launch_file)),
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
                _include('radio_vio', 'radio_stack.launch.py', {
                    'preset': LaunchConfiguration('radio_preset'),
                    'decoder_mode': LaunchConfiguration('decoder_mode'),
                    'gain': LaunchConfiguration('rtl_gain'),
                    'device_index': LaunchConfiguration('rtl_device_index'),
                }),
            ],
        ))

    # Stage 3: cameras. Grasshopper stereo is the canonical camera stage.
    camera_actions = []
    if _bool_arg(context, 'enable_grasshopper'):
        camera_actions.extend(_optional_stage(context, 'spinnaker_camera_driver', 'grasshopper_stereo_min.launch.py', {
            'left_serial': LaunchConfiguration('grasshopper_left_serial'),
            'right_serial': LaunchConfiguration('grasshopper_right_serial'),
            'parameter_file': LaunchConfiguration('grasshopper_parameter_file'),
            'left_parameter_file': LaunchConfiguration('grasshopper_left_parameter_file'),
            'right_parameter_file': LaunchConfiguration('grasshopper_right_parameter_file'),
            'left_camera_info_url': LaunchConfiguration('grasshopper_left_camera_info_url'),
            'right_camera_info_url': LaunchConfiguration('grasshopper_right_camera_info_url'),
        }))
    if _bool_arg(context, 'enable_realsense'):
        camera_actions.extend(_optional_stage(context, 'realsense2_camera', 'rs_launch.py'))

    if camera_actions:
        actions.append(TimerAction(
            period=_float_arg(context, 'camera_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 3: starting cameras'),
                *camera_actions,
            ],
        ))

    # Stage 4: spectrometer.
    if _bool_arg(context, 'enable_spectrometer'):
        actions.append(TimerAction(
            period=_float_arg(context, 'spectrometer_delay_sec'),
            actions=[
                LogInfo(msg='[earth_rover_system] Stage 4: starting spectrometer'),
                _include('spectrometery_ros2', 'spectrometer_data_publisher.launch.py', {
                    'plot_image': LaunchConfiguration('spectrometer_plot_image'),
                    'integration_time_micros': LaunchConfiguration('spectrometer_integration_time_micros'),
                    'publish_period_sec': LaunchConfiguration('spectrometer_publish_period_sec'),
                }),
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
        DeclareLaunchArgument('gcs_url', default_value='udp://192.168.1.7:14550'),
        DeclareLaunchArgument('mavros_namespace', default_value='mavros'),

        DeclareLaunchArgument('enable_radio_vio', default_value='true'),
        DeclareLaunchArgument('radio_delay_sec', default_value='8.0'),
        DeclareLaunchArgument('radio_preset', default_value='full'),
        DeclareLaunchArgument('decoder_mode', default_value='dump1090'),
        DeclareLaunchArgument('rtl_gain', default_value='-1.0'),
        DeclareLaunchArgument('rtl_device_index', default_value='0'),

        DeclareLaunchArgument('camera_delay_sec', default_value='16.0'),
        DeclareLaunchArgument('enable_grasshopper', default_value='true'),
        DeclareLaunchArgument('grasshopper_left_serial', default_value='22312692'),
        DeclareLaunchArgument('grasshopper_right_serial', default_value='22312674'),
        DeclareLaunchArgument('grasshopper_parameter_file', default_value='grasshopper.yaml'),
        DeclareLaunchArgument('grasshopper_left_parameter_file', default_value=''),
        DeclareLaunchArgument('grasshopper_right_parameter_file', default_value=''),
        DeclareLaunchArgument('grasshopper_left_camera_info_url', default_value=''),
        DeclareLaunchArgument('grasshopper_right_camera_info_url', default_value=''),
        DeclareLaunchArgument('enable_realsense', default_value='false'),

        DeclareLaunchArgument('enable_spectrometer', default_value='true'),
        DeclareLaunchArgument('spectrometer_delay_sec', default_value='24.0'),
        DeclareLaunchArgument('spectrometer_plot_image', default_value='true'),
        DeclareLaunchArgument('spectrometer_integration_time_micros', default_value='500000'),
        DeclareLaunchArgument('spectrometer_publish_period_sec', default_value='0.1'),

        OpaqueFunction(function=launch_setup),
    ])
