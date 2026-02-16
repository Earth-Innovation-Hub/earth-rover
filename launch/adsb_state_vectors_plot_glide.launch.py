#!/usr/bin/env python3
"""
Launch the glide profile plot (altitude vs path angle, speed arrows).

Requires the aircraft state vectors node (publishing JSON on
/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors).
Publishes sensor_msgs/Image on ~/aircraft_plot_glide_image.

Usage:
  ros2 launch deepgis_vehicles adsb_state_vectors_plot_glide.launch.py

  ros2 launch deepgis_vehicles adsb_state_vectors_plot_glide.launch.py glide_angle_range_deg:=10.0
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'state_vectors_topic',
        default_value='/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors',
        description='Topic for aircraft state vectors JSON',
    )
    range_arg = DeclareLaunchArgument(
        'glide_angle_range_deg',
        default_value='8.0',
        description='Half-range of glide angle on X axis (degrees)',
    )
    alt_max_arg = DeclareLaunchArgument(
        'altitude_max_ft',
        default_value='45000.0',
        description='Maximum altitude for Y axis (ft)',
    )
    pkg_prefix = get_package_prefix('deepgis_vehicles')
    plot_script = os.path.join(pkg_prefix, 'lib', 'deepgis_vehicles', 'adsb_state_vectors_plot_glide.py')

    plot_node = Node(
        package='deepgis_vehicles',
        executable=plot_script,
        name='adsb_state_vectors_plot_glide',
        parameters=[{
            'state_vectors_topic': LaunchConfiguration('state_vectors_topic'),
            'glide_angle_range_deg': LaunchConfiguration('glide_angle_range_deg'),
            'altitude_max_ft': LaunchConfiguration('altitude_max_ft'),
            'publish_rate_hz': 5.0,
            'image_width': 800,
            'image_height': 800,
            'speed_arrow_scale': 0.008,
            'show_labels': True,
        }],
        output='screen',
    )

    return LaunchDescription([topic_arg, range_arg, alt_max_arg, plot_node])
