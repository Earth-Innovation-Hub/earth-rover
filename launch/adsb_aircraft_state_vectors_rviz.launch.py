#!/usr/bin/env python3
"""
Launch ADS-B aircraft state vectors + RViz visualization.

Starts:
  1. adsb_aircraft_state_vectors_node (JSON state vectors from MAVROS + ADS-B)
  2. adsb_state_vectors_to_rviz_node (PoseArray + MarkerArray for RViz)
  3. RViz2 with config for aircraft poses and velocity arrows

Requires MAVROS and ADS-B aircraft list to be running (e.g. rtl_adsb stack).

Usage:
  ros2 launch deepgis_vehicles adsb_aircraft_state_vectors_rviz.launch.py

  # Without launching state vectors (if already running)
  ros2 launch deepgis_vehicles adsb_aircraft_state_vectors_rviz.launch.py launch_state_vectors:=false

  # Custom fixed frame (must match MAVROS local frame)
  ros2 launch deepgis_vehicles adsb_aircraft_state_vectors_rviz.launch.py frame_id:=odom
"""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('deepgis_vehicles')
    pkg_prefix = get_package_prefix('deepgis_vehicles')
    rviz_config = os.path.join(pkg_share, 'adsb_state_vectors.rviz')
    # Use full path to converter script so launch finds it after install
    to_rviz_script = os.path.join(pkg_prefix, 'lib', 'deepgis_vehicles', 'adsb_state_vectors_to_rviz_node.py')

    launch_state_vectors_arg = DeclareLaunchArgument(
        'launch_state_vectors',
        default_value='true',
        description='Launch the state vectors node (set false if already running)',
    )
    mavros_ns_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='/mavros',
        description='MAVROS namespace',
    )
    aircraft_list_arg = DeclareLaunchArgument(
        'aircraft_list_topic',
        default_value='/adsb/rtl_adsb_decoder_node/aircraft_list',
        description='Topic for aircraft list JSON',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Fixed frame for RViz (should match MAVROS local frame, e.g. map or odom)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    state_vectors_node = Node(
        package='deepgis_vehicles',
        executable='adsb_aircraft_state_vectors_node.py',
        name='adsb_aircraft_state_vectors_node',
        namespace='adsb',
        parameters=[{
            'mavros_namespace': LaunchConfiguration('mavros_namespace'),
            'aircraft_list_topic': LaunchConfiguration('aircraft_list_topic'),
            'publish_rate_hz': 5.0,
            'mavros_local_frame': 'ned',
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_state_vectors')),
    )

    to_rviz_node = Node(
        package='deepgis_vehicles',
        executable=to_rviz_script,
        name='adsb_state_vectors_to_rviz_node',
        namespace='adsb',
        parameters=[{
            'state_vectors_topic': '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors',
            'frame_id': LaunchConfiguration('frame_id'),
            'velocity_arrow_scale': 0.5,
            'velocity_arrow_min_length': 2.0,
            'show_labels': True,
        }],
        output='screen',
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        launch_state_vectors_arg,
        mavros_ns_arg,
        aircraft_list_arg,
        frame_id_arg,
        use_sim_time_arg,
        state_vectors_node,
        to_rviz_node,
        rviz_node,
    ])
