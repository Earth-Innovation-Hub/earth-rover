#!/usr/bin/env python3
"""
Single-stack launcher for RTL ADS-B, aircraft state vectors, and landmark VO plot.

Use one launch argument, ``preset``, to choose which pieces start:

  decoder
      ``rtl_adsb.launch.py`` only (RTL-SDR + decoder).
  decoder_state_vectors
      Adds ``adsb_aircraft_state_vectors`` (needs MAVROS for meaningful framing).
  decoder_landmark
      Adds ``landmark_vo_plot_2d`` (2D VO / traffic overlay plot).
  full
      ADS-B base plus state vectors and landmark plot (default).

  The RTL/ADS-B base always starts; ``preset`` only selects extra nodes.

Examples:

  ros2 launch radio_vio radio_stack.launch.py
  ros2 launch radio_vio radio_stack.launch.py preset:=decoder
  ros2 launch radio_vio radio_stack.launch.py preset:=full gain:=42.0 decoder_mode:=iq
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def _pkg_launch(name: str) -> str:
    return os.path.join(get_package_share_directory('radio_vio'), 'launch', name)


def generate_launch_description():
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='full',
        description=(
            'Stack preset: decoder | decoder_state_vectors | '
            'decoder_landmark | full'
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

    include_vectors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('adsb_aircraft_state_vectors.launch.py')),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    preset,
                    "' in ('decoder_state_vectors', 'full')",
                ]
            )
        ),
    )

    include_landmark = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_pkg_launch('landmark_vo_plot_2d.launch.py')),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    preset,
                    "' in ('decoder_landmark', 'full')",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            preset_arg,
            gain_arg,
            decoder_mode_arg,
            device_index_arg,
            include_rtl,
            include_vectors,
            include_landmark,
        ]
    )
