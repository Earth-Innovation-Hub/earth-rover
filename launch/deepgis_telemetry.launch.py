#!/usr/bin/env python3
"""
Launch file for DeepGIS Telemetry Publisher

Launches the node that publishes MAVROS telemetry data to DeepGIS API.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    api_url_arg = DeclareLaunchArgument(
        'deepgis_api_url',
        default_value='https://deepgis.org',
        description='DeepGIS API base URL'
    )
    
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value='',
        description='API key for authentication (optional)'
    )
    
    asset_name_arg = DeclareLaunchArgument(
        'asset_name',
        default_value='MAVROS Vehicle',
        description='Asset/vehicle name'
    )
    
    session_id_arg = DeclareLaunchArgument(
        'session_id',
        default_value='',
        description='Session ID (auto-generated if empty)'
    )
    
    project_title_arg = DeclareLaunchArgument(
        'project_title',
        default_value='MAVROS Data Collection',
        description='Project title'
    )
    
    flight_mode_arg = DeclareLaunchArgument(
        'flight_mode',
        default_value='AUTO',
        description='Flight mode (MANUAL, AUTO, GUIDED, etc.)'
    )
    
    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='Telemetry Collection',
        description='Mission type description'
    )
    
    mavros_namespace_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='/mavros',
        description='MAVROS namespace'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Telemetry publish rate in Hz'
    )
    
    batch_size_arg = DeclareLaunchArgument(
        'batch_size',
        default_value='10',
        description='Number of samples before batch upload'
    )
    
    enable_batch_arg = DeclareLaunchArgument(
        'enable_batch_mode',
        default_value='true',
        description='Enable batch mode (true) or real-time mode (false)'
    )
    
    # DeepGIS Telemetry Publisher Node
    telemetry_node = Node(
        package='deepgis_vehicles',
        executable='deepgis_telemetry_publisher.py',
        name='deepgis_telemetry_publisher',
        output='screen',
        parameters=[{
            'deepgis_api_url': LaunchConfiguration('deepgis_api_url'),
            'api_key': LaunchConfiguration('api_key'),
            'asset_name': LaunchConfiguration('asset_name'),
            'session_id': LaunchConfiguration('session_id'),
            'project_title': LaunchConfiguration('project_title'),
            'flight_mode': LaunchConfiguration('flight_mode'),
            'mission_type': LaunchConfiguration('mission_type'),
            'mavros_namespace': LaunchConfiguration('mavros_namespace'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'batch_size': LaunchConfiguration('batch_size'),
            'enable_batch_mode': LaunchConfiguration('enable_batch_mode'),
        }]
    )
    
    return LaunchDescription([
        api_url_arg,
        api_key_arg,
        asset_name_arg,
        session_id_arg,
        project_title_arg,
        flight_mode_arg,
        mission_type_arg,
        mavros_namespace_arg,
        publish_rate_arg,
        batch_size_arg,
        enable_batch_arg,
        telemetry_node,
    ])

