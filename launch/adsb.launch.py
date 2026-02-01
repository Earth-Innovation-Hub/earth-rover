#!/usr/bin/env python3
"""
ADS-B Decoder Launch File

Launches the ADS-B decoder node with HydraSDR configured for 1090 MHz.
ADS-B (Automatic Dependent Surveillance-Broadcast) is used by aircraft
to broadcast position, velocity, and identification information.

Usage:
  # Basic ADS-B decoding
  ros2 launch deepgis_vehicles adsb.launch.py

  # With custom SDR configuration
  ros2 launch deepgis_vehicles adsb.launch.py \
    hydra_frequency:=1090.0e6 \
    hydra_sample_rate:=10.0e6 \
    hydra_gain:=30

  # Enable raw message publishing
  ros2 launch deepgis_vehicles adsb.launch.py publish_raw_messages:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # ====================================================================
    # Launch Arguments
    # ====================================================================
    
    # SDR Configuration (for HydraSDR)
    hydra_freq_arg = DeclareLaunchArgument(
        'hydra_frequency',
        default_value='1090.0e6',
        description='SDR center frequency in Hz (1090 MHz for ADS-B)'
    )
    
    hydra_sample_rate_arg = DeclareLaunchArgument(
        'hydra_sample_rate',
        default_value='10.0e6',
        description='SDR sample rate in Hz (10 MSPS recommended for ADS-B)'
    )
    
    hydra_gain_arg = DeclareLaunchArgument(
        'hydra_gain',
        default_value='20',
        description='SDR gain (0-21 dB)'
    )
    
    # ADS-B Decoder Configuration
    iq_topic_arg = DeclareLaunchArgument(
        'iq_topic',
        default_value='/hydra_sdr/hydra_sdr_node/iq_samples',
        description='Topic for IQ samples from SDR'
    )
    
    enable_decoding_arg = DeclareLaunchArgument(
        'enable_decoding',
        default_value='true',
        description='Enable ADS-B message decoding'
    )
    
    publish_raw_arg = DeclareLaunchArgument(
        'publish_raw_messages',
        default_value='false',
        description='Publish raw decoded messages for debugging'
    )
    
    max_aircraft_arg = DeclareLaunchArgument(
        'max_aircraft',
        default_value='100',
        description='Maximum number of aircraft to track'
    )
    
    aircraft_timeout_arg = DeclareLaunchArgument(
        'aircraft_timeout',
        default_value='60.0',
        description='Timeout in seconds for removing stale aircraft'
    )
    
    enable_visualizer_arg = DeclareLaunchArgument(
        'enable_visualizer',
        default_value='true',
        description='Enable SDR visualizer for spectrum and waterfall display'
    )
    
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='Enable debug messages for ADS-B detection'
    )
    
    # ====================================================================
    # Startup Message
    # ====================================================================
    
    startup_msg = LogInfo(msg="""
╔════════════════════════════════════════════════════════════╗
║              Earth Rover - ADS-B Decoder                    ║
╠════════════════════════════════════════════════════════════╣
║  Frequency: 1090 MHz (ADS-B)                               ║
║  Protocol: Mode S Extended Squitter                        ║
║  Data: Aircraft position, velocity, identification        ║
╠════════════════════════════════════════════════════════════╣
║  Features:                                                 ║
║    - Real-time aircraft tracking                           ║
║    - Position, velocity, and callsign decoding            ║
║    - Multiple aircraft support                             ║
║    - Automatic stale aircraft cleanup                      ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    # ====================================================================
    # HydraSDR Node (configured for ADS-B)
    # ====================================================================
    
    hydra_sdr_node = Node(
        package='deepgis_vehicles',
        executable='hydra_sdr_node.py',
        name='hydra_sdr_node',
        namespace='hydra_sdr',
        parameters=[{
            'auto_connect': True,
            'auto_stream': True,
            'center_frequency': LaunchConfiguration('hydra_frequency'),
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'gain': LaunchConfiguration('hydra_gain'),
            'publish_raw_iq': True,
            'publish_spectrum': True,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # ====================================================================
    # Spectrum Analyzer Node (for waterfall display)
    # ====================================================================
    
    spectrum_analyzer_node = Node(
        package='deepgis_vehicles',
        executable='spectrum_analyzer_node.py',
        name='spectrum_analyzer_node',
        namespace='hydra_sdr',
        parameters=[{
            'fft_size': 2048,
            'window_type': 'hanning',
            'averaging': 4,
            'waterfall_depth': 200,
            'peak_threshold_db': 10.0,
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'center_frequency': LaunchConfiguration('hydra_frequency'),
        }],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('~/iq_samples', '/hydra_sdr/hydra_sdr_node/iq_samples'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_visualizer')),
    )
    
    # ====================================================================
    # ADS-B Decoder Node
    # ====================================================================
    
    adsb_decoder_node = Node(
        package='deepgis_vehicles',
        executable='adsb_decoder_node.py',
        name='adsb_decoder_node',
        namespace='adsb',
        parameters=[{
            'iq_topic': LaunchConfiguration('iq_topic'),
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'center_frequency': LaunchConfiguration('hydra_frequency'),
            'enable_decoding': LaunchConfiguration('enable_decoding'),
            'publish_raw_messages': LaunchConfiguration('publish_raw_messages'),
            'max_aircraft': LaunchConfiguration('max_aircraft'),
            'aircraft_timeout': LaunchConfiguration('aircraft_timeout'),
            'debug': LaunchConfiguration('enable_debug'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # ====================================================================
    # SDR Visualizer Node
    # ====================================================================
    
    sdr_visualizer_node = Node(
        package='deepgis_vehicles',
        executable='sdr_visualizer.py',
        name='sdr_visualizer',
        namespace='sdr_viz',
        parameters=[{
            'spectrum_topic': '/hydra_sdr/hydra_sdr_node/spectrum',
            'waterfall_topic': '/hydra_sdr/spectrum_analyzer_node/waterfall',
            'center_frequency': LaunchConfiguration('hydra_frequency'),
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'show_controls': True,
            'sdr_node_name': '/hydra_sdr/hydra_sdr_node',
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_visualizer')),
    )
    
    # ====================================================================
    # Return Launch Description
    # ====================================================================
    
    return LaunchDescription([
        # SDR configuration
        hydra_freq_arg,
        hydra_sample_rate_arg,
        hydra_gain_arg,
        
        # ADS-B decoder configuration
        iq_topic_arg,
        enable_decoding_arg,
        publish_raw_arg,
        max_aircraft_arg,
        aircraft_timeout_arg,
        enable_visualizer_arg,
        enable_debug_arg,
        
        # Launch components
        startup_msg,
        hydra_sdr_node,
        spectrum_analyzer_node,
        adsb_decoder_node,
        sdr_visualizer_node,
    ])

