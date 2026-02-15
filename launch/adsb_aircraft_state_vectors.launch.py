#!/usr/bin/env python3
"""
Launch ADS-B Aircraft State Vectors node.

Publishes aircraft state vectors (position, orientation, linear/angular velocity)
in the same local UTM/ENU frame as the trike (from MAVROS). Requires MAVROS and
ADS-B aircraft list to be running.

Usage:
  ros2 launch deepgis_vehicles adsb_aircraft_state_vectors.launch.py

  # Custom topics / frame
  ros2 launch deepgis_vehicles adsb_aircraft_state_vectors.launch.py \
    aircraft_list_topic:=/adsb/rtl_adsb_decoder_node/aircraft_list \
    mavros_local_frame:=enu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mavros_ns_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='/mavros',
        description='MAVROS namespace',
    )
    aircraft_list_arg = DeclareLaunchArgument(
        'aircraft_list_topic',
        default_value='/adsb/rtl_adsb_decoder_node/aircraft_list',
        description='Topic for aircraft list JSON (flight + radio)',
    )
    rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='5.0',
        description='Publish rate for state vectors (Hz)',
    )
    frame_arg = DeclareLaunchArgument(
        'mavros_local_frame',
        default_value='ned',
        description="MAVROS local frame: 'ned' or 'enu'; output is always ENU",
    )

    state_vectors_node = Node(
        package='deepgis_vehicles',
        executable='adsb_aircraft_state_vectors_node.py',
        name='adsb_aircraft_state_vectors_node',
        namespace='adsb',
        parameters=[{
            'mavros_namespace': LaunchConfiguration('mavros_namespace'),
            'aircraft_list_topic': LaunchConfiguration('aircraft_list_topic'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'mavros_local_frame': LaunchConfiguration('mavros_local_frame'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        mavros_ns_arg,
        aircraft_list_arg,
        rate_arg,
        frame_arg,
        state_vectors_node,
    ])
