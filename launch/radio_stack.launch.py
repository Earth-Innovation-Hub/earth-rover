#!/usr/bin/env python3
"""
Compatibility entry point: same as ``radio_vio/radio_stack.launch.py``.

Prefer::

  ros2 launch radio_vio radio_stack.launch.py preset:=full

Use this package only if you still run ``deepgis_vehicles`` as the primary launch prefix.
Forwarded args: preset, gain, decoder_mode, device_index (see radio_vio launch file).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'preset',
                default_value='full',
                description=(
                    'decoder | state | plots | vio | full '
                    '(aliases: decoder_state_vectors, decoder_landmark)'
                ),
            ),
            DeclareLaunchArgument('gain', default_value='-1.0'),
            DeclareLaunchArgument('decoder_mode', default_value='dump1090'),
            DeclareLaunchArgument('device_index', default_value='0'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('radio_vio'),
                        'launch',
                        'radio_stack.launch.py',
                    )
                ),
                launch_arguments=[
                    ('preset', LaunchConfiguration('preset')),
                    ('gain', LaunchConfiguration('gain')),
                    ('decoder_mode', LaunchConfiguration('decoder_mode')),
                    ('device_index', LaunchConfiguration('device_index')),
                ],
            ),
        ]
    )
