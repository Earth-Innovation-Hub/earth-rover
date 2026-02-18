#!/usr/bin/env python3
"""
Launch the 2D landmark VO plot as a published ROS image.

Subscribes to pose (odometry), landmarks, and landmark observations.
Uses Grasshopper left (Chameleon narrow) camera model for predicted projections.
Publishes sensor_msgs/Image on ~/landmark_vo_plot_image (view with rqt_image_view).

Usage:
  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py

  # Demo mode (synthetic data when no topics)
  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py demo_mode:=true

  # Custom odom topic (e.g. vehicle/odometry)
  ros2 launch deepgis_vehicles landmark_vo_plot_2d.launch.py odom_topic:=/vehicle/odometry
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/mavros/local_position/odom',
        description='Odometry topic (nav_msgs/Odometry)'
    )
    estimated_position_arg = DeclareLaunchArgument(
        'estimated_position_topic', default_value='/adsb/rtl_adsb_decoder_node/estimated_position',
        description='ADS-B KF estimated position (JSON) for predictions'
    )
    landmarks_arg = DeclareLaunchArgument(
        'landmarks_topic', default_value='/vo/landmarks',
        description='Landmarks JSON topic'
    )
    observations_arg = DeclareLaunchArgument(
        'observations_topic', default_value='/vo/landmark_observations',
        description='Landmark observations JSON topic'
    )
    demo_arg = DeclareLaunchArgument(
        'demo_mode', default_value='false',
        description='Use synthetic landmarks/observations when no real data'
    )
    config_arg = DeclareLaunchArgument(
        'config_file', default_value='',
        description='Path to YAML config (optional)'
    )

    pkg_prefix = get_package_prefix('deepgis_vehicles')
    plot_script = os.path.join(pkg_prefix, 'lib', 'deepgis_vehicles', 'landmark_vo_plot_2d.py')

    # Parameters: prefer config_file if provided, else package YAML + launch overrides
    try:
        pkg_share = get_package_share_directory('deepgis_vehicles')
        default_config = os.path.join(pkg_share, 'config', 'landmark_vo_plot_2d.yaml')
    except Exception:
        default_config = None

    parameters = [default_config] if default_config and os.path.isfile(default_config) else []
    parameters.append({
            'odom_topic': LaunchConfiguration('odom_topic'),
            'estimated_position_topic': LaunchConfiguration('estimated_position_topic'),
            'landmarks_topic': LaunchConfiguration('landmarks_topic'),
            'observations_topic': LaunchConfiguration('observations_topic'),
            'demo_mode': LaunchConfiguration('demo_mode'),
            'publish_rate_hz': 5.0,
            'image_width': 800,
            'image_height': 800,
            'camera_fx': 1000.0,
            'camera_fy': 1000.0,
            'camera_cx': 640.0,
            'camera_cy': 512.0,
            'camera_width': 1280,
            'camera_height': 1024,
            'cam_height': 1.2,
            'adsb_state_vectors_topic': '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors',
            'enable_aircraft': True,
            'aircraft_max_range_km': 8.0,
            'aircraft_min_range_m': 50.0,
            'world_view_extent_km': 20.0,
            'mavros_local_frame': 'enu',
        })

    plot_node = Node(
        package='deepgis_vehicles',
        executable=plot_script,
        name='landmark_vo_plot_2d',
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        odom_arg, estimated_position_arg, landmarks_arg, observations_arg, demo_arg, config_arg,
        plot_node,
    ])
