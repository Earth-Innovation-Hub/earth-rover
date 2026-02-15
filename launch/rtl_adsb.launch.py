#!/usr/bin/env python3
"""
RTL-SDR V4 ADS-B Decoder Launch File

Launches ADS-B aircraft tracking using the RTL-SDR Blog V4 at 1090 MHz.
Supports two decoding backends:
  1. dump1090 (recommended): High-performance C decoder, de-facto standard
  2. IQ mode (fallback): Python-based decoding via pyModeS

Usage:
  # Default -- dump1090 mode, auto-gain
  ros2 launch deepgis_vehicles rtl_adsb.launch.py

  # With manual gain and raw message output
  ros2 launch deepgis_vehicles rtl_adsb.launch.py gain:=42.0 publish_raw_messages:=true

  # IQ decoding mode (no dump1090 required, uses rtl_sdr_node + pyModeS)
  ros2 launch deepgis_vehicles rtl_adsb.launch.py decoder_mode:=iq

  # With visualizer for spectrum monitoring
  ros2 launch deepgis_vehicles rtl_adsb.launch.py enable_visualizer:=true

  # Second RTL-SDR dongle
  ros2 launch deepgis_vehicles rtl_adsb.launch.py device_index:=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ====================================================================
    # Launch Arguments
    # ====================================================================

    # SDR configuration
    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='-1.0',
        description='Tuner gain in dB (-1 for auto, 0-49.6 for manual)'
    )

    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='RTL-SDR device index'
    )

    ppm_arg = DeclareLaunchArgument(
        'ppm_correction',
        default_value='0',
        description='Frequency correction in PPM'
    )

    bias_tee_arg = DeclareLaunchArgument(
        'bias_tee',
        default_value='false',
        description='Enable 4.5V bias tee'
    )

    # ADS-B decoder configuration
    decoder_mode_arg = DeclareLaunchArgument(
        'decoder_mode',
        default_value='dump1090',
        description='Decoder backend: dump1090 (recommended) or iq (pyModeS)'
    )

    dump1090_path_arg = DeclareLaunchArgument(
        'dump1090_path',
        default_value='dump1090',
        description='Path to dump1090 binary'
    )

    dump1090_http_port_arg = DeclareLaunchArgument(
        'dump1090_http_port',
        default_value='8082',
        description='dump1090 web map HTTP port (default 8082 to avoid conflict with 8080/8081)'
    )

    publish_raw_arg = DeclareLaunchArgument(
        'publish_raw_messages',
        default_value='false',
        description='Publish raw decoded messages for debugging'
    )

    max_aircraft_arg = DeclareLaunchArgument(
        'max_aircraft',
        default_value='200',
        description='Maximum number of aircraft to track'
    )

    aircraft_timeout_arg = DeclareLaunchArgument(
        'aircraft_timeout',
        default_value='60.0',
        description='Timeout in seconds for removing stale aircraft'
    )

    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug logging for decoder'
    )

    # Visualizer / spectrum analyzer
    enable_visualizer_arg = DeclareLaunchArgument(
        'enable_visualizer',
        default_value='false',
        description='Enable SDR spectrum visualizer'
    )

    enable_spectrum_arg = DeclareLaunchArgument(
        'enable_spectrum_analyzer',
        default_value='true',
        description='Enable spectrum analyzer node'
    )

    # ====================================================================
    # Startup Banner
    # ====================================================================

    startup_msg = LogInfo(msg="""
╔════════════════════════════════════════════════════════════╗
║        Earth Rover - RTL-SDR V4 ADS-B Decoder              ║
╠════════════════════════════════════════════════════════════╣
║  Frequency: 1090 MHz (ADS-B)                               ║
║  Protocol: Mode S Extended Squitter                        ║
║  Hardware: RTL-SDR Blog V4                                  ║
╠════════════════════════════════════════════════════════════╣
║  Modes:                                                    ║
║    dump1090 - High-performance C decoder (recommended)     ║
║    iq       - Python pyModeS decoder (fallback)            ║
║  Features:                                                 ║
║    - Real-time aircraft tracking                           ║
║    - Position, velocity, callsign decoding                 ║
║    - SBS BaseStation format parsing                        ║
╚════════════════════════════════════════════════════════════╝
    """)

    # ====================================================================
    # RTL-SDR V4 Driver Node (for IQ mode and spectrum analyzer)
    # Runs with 1090 MHz, 2.4 MSPS for ADS-B
    # ====================================================================

    rtl_sdr_node = Node(
        package='deepgis_vehicles',
        executable='rtl_sdr_node.py',
        name='rtl_sdr_node',
        namespace='rtl_sdr',
        parameters=[{
            'auto_connect': True,
            'auto_stream': True,
            'center_frequency': 1090.0e6,
            'sample_rate': 2.4e6,
            'gain': LaunchConfiguration('gain'),
            'bias_tee': LaunchConfiguration('bias_tee'),
            'ppm_correction': LaunchConfiguration('ppm_correction'),
            'device_index': LaunchConfiguration('device_index'),
            'publish_raw_iq': True,
            'publish_spectrum': True,
        }],
        output='screen',
        emulate_tty=True,
        # In dump1090 mode, dump1090 owns the dongle, so don't launch
        # the rtl_sdr_node driver (it would conflict).
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('decoder_mode'), "' == 'iq'"
            ])
        ),
    )

    # ====================================================================
    # Spectrum Analyzer Node
    # ====================================================================

    spectrum_analyzer_node = Node(
        package='deepgis_vehicles',
        executable='spectrum_analyzer_node.py',
        name='spectrum_analyzer_node',
        namespace='rtl_sdr',
        parameters=[{
            'fft_size': 2048,
            'window_type': 'hanning',
            'averaging': 4,
            'waterfall_depth': 200,
            'peak_threshold_db': 10.0,
            'sample_rate': 2.4e6,
            'center_frequency': 1090.0e6,
        }],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('~/iq_samples', '/rtl_sdr/rtl_sdr_node/iq_samples'),
        ],
        # Only in IQ mode (dump1090 owns the dongle in dump1090 mode)
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('decoder_mode'), "' == 'iq'"
                " and '", LaunchConfiguration('enable_spectrum_analyzer'), "' == 'true'"
            ])
        ),
    )

    # ====================================================================
    # ADS-B Decoder Node
    # ====================================================================

    adsb_decoder_node = Node(
        package='deepgis_vehicles',
        executable='rtl_adsb_decoder_node.py',
        name='rtl_adsb_decoder_node',
        namespace='adsb',
        parameters=[{
            'mode': LaunchConfiguration('decoder_mode'),
            'dump1090_path': LaunchConfiguration('dump1090_path'),
            'dump1090_http_port': LaunchConfiguration('dump1090_http_port'),
            'iq_topic': '/rtl_sdr/rtl_sdr_node/iq_samples',
            'device_index': LaunchConfiguration('device_index'),
            'gain': LaunchConfiguration('gain'),
            'publish_raw_messages': LaunchConfiguration('publish_raw_messages'),
            'max_aircraft': LaunchConfiguration('max_aircraft'),
            'aircraft_timeout': LaunchConfiguration('aircraft_timeout'),
            'debug': LaunchConfiguration('enable_debug'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ====================================================================
    # SDR Visualizer (only in IQ mode)
    # ====================================================================

    sdr_visualizer_node = Node(
        package='deepgis_vehicles',
        executable='sdr_visualizer.py',
        name='sdr_visualizer',
        namespace='rtl_sdr_viz',
        parameters=[{
            'spectrum_topic': '/rtl_sdr/rtl_sdr_node/spectrum',
            'waterfall_topic': '/rtl_sdr/spectrum_analyzer_node/waterfall',
            'center_frequency': 1090.0e6,
            'sample_rate': 2.4e6,
            'show_controls': True,
            'sdr_node_name': '/rtl_sdr/rtl_sdr_node',
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_visualizer')),
    )

    # ====================================================================
    # Return Launch Description
    # ====================================================================

    return LaunchDescription([
        # SDR config
        gain_arg,
        device_index_arg,
        ppm_arg,
        bias_tee_arg,

        # Decoder config
        decoder_mode_arg,
        dump1090_path_arg,
        dump1090_http_port_arg,
        publish_raw_arg,
        max_aircraft_arg,
        aircraft_timeout_arg,
        enable_debug_arg,

        # Visualizer/spectrum
        enable_visualizer_arg,
        enable_spectrum_arg,

        # Nodes
        startup_msg,
        rtl_sdr_node,
        spectrum_analyzer_node,
        adsb_decoder_node,
        sdr_visualizer_node,
    ])
