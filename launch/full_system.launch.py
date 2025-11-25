#!/usr/bin/env python3
"""
Full System Launch File

Launches MAVROS, Vehicle Interface, and DeepGIS Telemetry Publisher together.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('deepgis_vehicles')
    
    # Launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0:921600',
        description='FCU connection URL'
    )
    
    asset_name_arg = DeclareLaunchArgument(
        'asset_name',
        default_value='MAVROS Vehicle',
        description='Asset/vehicle name for DeepGIS'
    )
    
    session_id_arg = DeclareLaunchArgument(
        'session_id',
        default_value='',
        description='Session ID (auto-generated if empty)'
    )
    
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value='',
        description='DeepGIS API key (optional)'
    )
    
    deepgis_api_url_arg = DeclareLaunchArgument(
        'deepgis_api_url',
        default_value='http://192.168.0.186:8080',
        description='DeepGIS API URL'
    )
    
    enable_telemetry_arg = DeclareLaunchArgument(
        'enable_telemetry',
        default_value='true',
        description='Enable DeepGIS telemetry publishing'
    )
    
    # Include vehicle interface launch (MAVROS + Vehicle Interface Node)
    vehicle_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'vehicle_interface.launch.py')
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
        }.items()
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
            'project_title': 'MAVROS Data Collection',
            'flight_mode': 'AUTO',
            'mission_type': 'Telemetry Collection',
            'mavros_namespace': '/mavros',
            'publish_rate': 1.0,
            'batch_size': 10,
            'enable_batch_mode': True,
        }],
        condition=IfCondition(LaunchConfiguration('enable_telemetry'))
    )
    
    return LaunchDescription([
        fcu_url_arg,
        asset_name_arg,
        session_id_arg,
        api_key_arg,
        deepgis_api_url_arg,
        enable_telemetry_arg,
        vehicle_interface_launch,
        telemetry_node,
    ])

