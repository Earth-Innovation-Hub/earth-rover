#!/usr/bin/env python3
"""
Launch the 2D aircraft-state plot as a published ROS image (no GUI).

Requires the aircraft state vectors node (publishing JSON on
/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors).
Publishes sensor_msgs/Image on ~/aircraft_plot_image (view with rqt_image_view).

Usage:
  ros2 launch deepgis_vehicles adsb_state_vectors_plot_2d.launch.py

  # Custom range (e.g. 50 km)
  ros2 launch deepgis_vehicles adsb_state_vectors_plot_2d.launch.py range_km:=50.0
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    range_arg = DeclareLaunchArgument('range_km', default_value='100.0', description='Half-width of plot in km')
    topic_arg = DeclareLaunchArgument('state_vectors_topic', default_value='/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors')
    pkg_prefix = get_package_prefix('deepgis_vehicles')
    plot_script = os.path.join(pkg_prefix, 'lib', 'deepgis_vehicles', 'adsb_state_vectors_plot_2d.py')

    plot_node = Node(
        package='deepgis_vehicles',
        executable=plot_script,
        name='adsb_state_vectors_plot_2d',
        parameters=[{
            'state_vectors_topic': LaunchConfiguration('state_vectors_topic'),
            'range_km': LaunchConfiguration('range_km'),
            'publish_rate_hz': 5.0,
            'image_width': 800,
            'image_height': 800,
            'show_velocity_arrows': True,
            'velocity_arrow_scale': 80.0,
            'show_labels': True,
        }],
        output='screen',
    )

    return LaunchDescription([range_arg, topic_arg, plot_node])
