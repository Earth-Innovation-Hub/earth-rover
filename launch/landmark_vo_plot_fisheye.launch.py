#!/usr/bin/env python3
"""
Launch the spherical fisheye plot node.

Publishes sensor_msgs/Image on ~/fisheye_panorama (view with rqt_image_view).
Pure spherical fisheye (equidistant): center=zenith, edge=horizon, 180° FOV.

Usage:
  ros2 launch deepgis_vehicles landmark_vo_plot_fisheye.launch.py
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_prefix('deepgis_vehicles')
    script = os.path.join(pkg_prefix, 'lib', 'deepgis_vehicles', 'landmark_vo_plot_fisheye.py')

    try:
        pkg_share = get_package_share_directory('deepgis_vehicles')
        default_config = os.path.join(pkg_share, 'config', 'landmark_vo_plot_fisheye.yaml')
    except Exception:
        default_config = None

    parameters = [default_config] if default_config and os.path.isfile(default_config) else []
    parameters.append({
        'odom_topic': '/mavros/local_position/odom',
        'estimated_position_topic': '/adsb/rtl_adsb_decoder_node/estimated_position',
        'landmarks_topic': '/vo/landmarks',
        'observations_topic': '/vo/landmark_observations',
        'publish_rate_hz': 5.0,
        'image_size': 512,
        'adsb_state_vectors_topic': '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors',
        'enable_aircraft': True,
        'aircraft_max_range_km': 8.0,
        'aircraft_min_range_m': 50.0,
        'aircraft_log_offset_scale': 12.0,
        'camera_hfov_deg': 65.2,
        'camera_vfov_deg': 54.2,
        'show_camera_footprint': True,
        'mavros_local_frame': 'enu',
    })

    node = Node(
        package='deepgis_vehicles',
        executable=script,
        name='landmark_vo_plot_fisheye',
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([node])
