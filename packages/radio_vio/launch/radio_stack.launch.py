#!/usr/bin/env python3
"""
Single-stack launcher for RTL ADS-B, aircraft state vectors, plots, and VIO.

Use one launch argument, ``preset``, to choose which pieces start:

  decoder
      ``rtl_adsb.launch.py`` only (RTL-SDR + decoder).
  state
      Adds ``adsb_aircraft_state_vectors`` (needs MAVROS for meaningful framing).
  plots
      Adds state vectors plus ADS-B 2D and glide image plots.
  vio
      Adds state vectors plus landmark VO 2D/fisheye visualizations.
  full
      ADS-B base, state vectors, ADS-B plots, and landmark VO plots (default).

  The RTL/ADS-B base always starts; ``preset`` only selects extra nodes.

Examples:

  ros2 launch radio_vio radio_stack.launch.py
  ros2 launch radio_vio radio_stack.launch.py preset:=decoder
  ros2 launch radio_vio radio_stack.launch.py preset:=full gain:=42.0 decoder_mode:=iq

Legacy preset aliases still accepted:
  decoder_state_vectors -> state
  decoder_landmark      -> vio
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _pkg_launch(name: str) -> str:
    return os.path.join(get_package_share_directory('radio_vio'), 'launch', name)


def _libexec(executable: str) -> str:
    return os.path.join(get_package_prefix('radio_vio'), 'lib', 'radio_vio', executable)


def _preset_condition(preset, names):
    return IfCondition(
        PythonExpression(
            [
                "'",
                preset,
                "' in (",
                ', '.join(repr(name) for name in names),
                ")",
            ]
        )
    )


def _optional_include(preset, names, launch_file: str, executable: str):
    condition = _preset_condition(preset, names)
    path = _pkg_launch(launch_file)
    exe_path = _libexec(executable)

    if os.path.isfile(path) and os.access(exe_path, os.X_OK):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            condition=condition,
        )

    reason = f'missing launch file {path}' if not os.path.isfile(path) else f'missing executable {exe_path}'
    return LogInfo(
        msg=f'[radio_stack] Skipping optional {launch_file}: {reason}',
        condition=condition,
    )


def generate_launch_description():
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='full',
        description=(
            'Stack preset: decoder | state | plots | vio | full '
            '(aliases: decoder_state_vectors, decoder_landmark)'
        ),
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='-1.0',
        description='Passed to rtl_adsb.launch.py',
    )
    decoder_mode_arg = DeclareLaunchArgument(
        'decoder_mode',
        default_value='dump1090',
        description='Passed to rtl_adsb.launch.py',
    )
    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='Passed to rtl_adsb.launch.py',
    )

    preset = LaunchConfiguration('preset')

    include_rtl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('rtl_adsb.launch.py')),
        launch_arguments=[
            ('gain', LaunchConfiguration('gain')),
            ('decoder_mode', LaunchConfiguration('decoder_mode')),
            ('device_index', LaunchConfiguration('device_index')),
        ],
    )

    include_vectors = _optional_include(
        preset,
        ('state', 'plots', 'vio', 'full', 'decoder_state_vectors', 'decoder_landmark'),
        'adsb_aircraft_state_vectors.launch.py',
        'adsb_aircraft_state_vectors_node.py',
    )

    include_plot_2d = _optional_include(
        preset,
        ('plots', 'full'),
        'adsb_state_vectors_plot_2d.launch.py',
        'adsb_state_vectors_plot_2d.py',
    )

    include_glide = _optional_include(
        preset,
        ('plots', 'full'),
        'adsb_state_vectors_plot_glide.launch.py',
        'adsb_state_vectors_plot_glide.py',
    )

    include_landmark_2d = _optional_include(
        preset,
        ('vio', 'full', 'decoder_landmark'),
        'landmark_vo_plot_2d.launch.py',
        'landmark_vo_plot_2d.py',
    )

    include_landmark_fisheye = _optional_include(
        preset,
        ('vio', 'full'),
        'landmark_vo_plot_fisheye.launch.py',
        'landmark_vo_plot_fisheye.py',
    )

    return LaunchDescription(
        [
            preset_arg,
            gain_arg,
            decoder_mode_arg,
            device_index_arg,
            include_rtl,
            include_vectors,
            include_plot_2d,
            include_glide,
            include_landmark_2d,
            include_landmark_fisheye,
        ]
    )
