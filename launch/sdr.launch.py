#!/usr/bin/env python3
"""
Earth Rover SDR Subsystem Launch

Launches SDR-related nodes for HydraSDR RFOne integration:
- Hydra SDR driver (using hydrasdr-host tools)
- Spectrum analyzer with waterfall and peak detection
- Optional: RTL-SDR driver
- Optional: SoapySDR interface

Based on:
- hydrasdr-host: https://github.com/hydrasdr/hydrasdr-host
- SoapyHydraSDR: https://github.com/hydrasdr/SoapyHydraSDR

Hardware Support:
- HydraSDR RFOne (24-1800 MHz, up to 10 MSPS)
- Sample types: Float32 IQ/Real, Int16 IQ/Real, Raw
- Gain control: LNA (0-15 dB), Mixer (0-15 dB), VGA (0-15 dB)
- Features: Bias-T for LNA power, PPB frequency calibration

Usage Examples:
  # Default - 433 MHz ISM band
  ros2 launch deepgis_vehicles sdr.launch.py

  # Custom frequency and gain
  ros2 launch deepgis_vehicles sdr.launch.py hydra_frequency:=915.0e6 hydra_gain:=30

  # Use SoapySDR backend
  ros2 launch deepgis_vehicles sdr.launch.py use_soapy:=true

  # High sample rate (10 MSPS)
  ros2 launch deepgis_vehicles sdr.launch.py hydra_sample_rate:=10.0e6

  # Enable bias tee for active antenna/LNA
  ros2 launch deepgis_vehicles sdr.launch.py bias_tee:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # ====================================================================
    # Launch Arguments - Device Selection
    # ====================================================================
    
    enable_hydra_arg = DeclareLaunchArgument(
        'enable_hydra', 
        default_value='true',
        description='Enable Hydra SDR RFOne'
    )
    
    enable_rtlsdr_arg = DeclareLaunchArgument(
        'enable_rtlsdr', 
        default_value='false',
        description='Enable RTL-SDR (alternative SDR)'
    )
    
    enable_spectrum_analyzer_arg = DeclareLaunchArgument(
        'enable_spectrum_analyzer', 
        default_value='true',
        description='Enable spectrum analyzer with waterfall and peak detection'
    )
    
    use_soapy_arg = DeclareLaunchArgument(
        'use_soapy',
        default_value='false',
        description='Use SoapySDR backend instead of direct hydrasdr tools'
    )
    
    # ====================================================================
    # Launch Arguments - HydraSDR Configuration
    # ====================================================================
    
    hydra_freq_arg = DeclareLaunchArgument(
        'hydra_frequency', 
        default_value='433.0e6',
        description='Hydra SDR center frequency in Hz (24-1800 MHz range)'
    )
    
    hydra_sample_rate_arg = DeclareLaunchArgument(
        'hydra_sample_rate', 
        default_value='2.5e6',
        description='Hydra SDR sample rate in Hz (2.5e6, 5e6, or 10e6)'
    )
    
    hydra_gain_arg = DeclareLaunchArgument(
        'hydra_gain', 
        default_value='20',
        description='Hydra SDR linearity gain (0-21 dB)'
    )
    
    lna_gain_arg = DeclareLaunchArgument(
        'lna_gain',
        default_value='8',
        description='Hydra SDR LNA gain (0-15 dB)'
    )
    
    mixer_gain_arg = DeclareLaunchArgument(
        'mixer_gain',
        default_value='8',
        description='Hydra SDR Mixer gain (0-15 dB)'
    )
    
    vga_gain_arg = DeclareLaunchArgument(
        'vga_gain',
        default_value='8',
        description='Hydra SDR VGA gain (0-15 dB)'
    )
    
    bias_tee_arg = DeclareLaunchArgument(
        'bias_tee',
        default_value='false',
        description='Enable 4.5V bias tee for LNA/active antenna power'
    )
    
    sample_type_arg = DeclareLaunchArgument(
        'sample_type',
        default_value='2',
        description='Sample type: 0=Float32IQ, 1=Float32Real, 2=Int16IQ (default), 3=Int16Real, 5=Raw'
    )
    
    serial_number_arg = DeclareLaunchArgument(
        'serial_number',
        default_value='',
        description='HydraSDR device serial number (leave empty for auto-detect)'
    )
    
    # ====================================================================
    # Launch Arguments - Spectrum Analyzer Configuration
    # ====================================================================
    
    fft_size_arg = DeclareLaunchArgument(
        'fft_size',
        default_value='2048',
        description='FFT size for spectrum analysis'
    )
    
    window_type_arg = DeclareLaunchArgument(
        'window_type',
        default_value='hanning',
        description='Window function: hanning, hamming, blackman, rectangular'
    )
    
    averaging_arg = DeclareLaunchArgument(
        'averaging',
        default_value='4',
        description='Number of spectra to average'
    )
    
    waterfall_depth_arg = DeclareLaunchArgument(
        'waterfall_depth',
        default_value='100',
        description='Waterfall display history depth'
    )
    
    peak_threshold_db_arg = DeclareLaunchArgument(
        'peak_threshold_db',
        default_value='10.0',
        description='Peak detection threshold above noise floor (dB)'
    )
    
    # ====================================================================
    # Launch Arguments - RTL-SDR Configuration
    # ====================================================================
    
    rtlsdr_freq_arg = DeclareLaunchArgument(
        'rtlsdr_frequency', 
        default_value='433.92e6',
        description='RTL-SDR center frequency (Hz)'
    )
    
    # ====================================================================
    # Launch Arguments - General
    # ====================================================================
    
    auto_stream_arg = DeclareLaunchArgument(
        'auto_stream',
        default_value='true',
        description='Automatically start streaming on launch (default: true)'
    )
    
    publish_raw_iq_arg = DeclareLaunchArgument(
        'publish_raw_iq',
        default_value='true',
        description='Publish raw IQ samples (high bandwidth!)'
    )
    
    # ====================================================================
    # Startup Message
    # ====================================================================
    
    startup_msg = LogInfo(msg="""
╔════════════════════════════════════════════════════════════╗
║         Earth Rover - HydraSDR RFOne Subsystem             ║
╠════════════════════════════════════════════════════════════╣
║  Hardware: HydraSDR RFOne Software-Defined Radio           ║
║  Frequency Range: 24-1800 MHz                              ║
║  Sample Rates: 2.5, 5, 10 MSPS                             ║
║  Gain: 0-45 dB (LNA + Mixer + VGA)                         ║
║  Features: Bias-T, PPB calibration, 12-bit ADC             ║
╠════════════════════════════════════════════════════════════╣
║  Software Stack:                                           ║
║    - libhydrasdr (C library)                               ║
║    - hydrasdr-host tools (CLI)                             ║
║    - SoapyHydraSDR (optional SoapySDR plugin)              ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    # ====================================================================
    # HydraSDR Node
    # ====================================================================
    
    hydra_sdr_node = Node(
        package='deepgis_vehicles',
        executable='hydra_sdr_node.py',
        name='hydra_sdr_node',
        namespace='hydra_sdr',
        parameters=[{
            'auto_connect': True,
            'auto_stream': LaunchConfiguration('auto_stream'),
            'center_frequency': LaunchConfiguration('hydra_frequency'),
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'gain': LaunchConfiguration('hydra_gain'),
            'lna_gain': LaunchConfiguration('lna_gain'),
            'mixer_gain': LaunchConfiguration('mixer_gain'),
            'vga_gain': LaunchConfiguration('vga_gain'),
            'bias_tee': LaunchConfiguration('bias_tee'),
            'sample_type': LaunchConfiguration('sample_type'),
            'publish_raw_iq': LaunchConfiguration('publish_raw_iq'),
            'publish_spectrum': True,
            'fft_size': LaunchConfiguration('fft_size'),
            'spectrum_averaging': LaunchConfiguration('averaging'),
            'use_soapy': LaunchConfiguration('use_soapy'),
            'serial_number': LaunchConfiguration('serial_number'),
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_hydra')),
    )
    
    # ====================================================================
    # Spectrum Analyzer Node
    # ====================================================================
    
    spectrum_analyzer_node = Node(
        package='deepgis_vehicles',
        executable='spectrum_analyzer_node.py',
        name='spectrum_analyzer_node',
        namespace='hydra_sdr',
        parameters=[{
            'fft_size': LaunchConfiguration('fft_size'),
            'window_type': LaunchConfiguration('window_type'),
            'averaging': LaunchConfiguration('averaging'),
            'waterfall_depth': LaunchConfiguration('waterfall_depth'),
            'peak_threshold_db': LaunchConfiguration('peak_threshold_db'),
            'sample_rate': LaunchConfiguration('hydra_sample_rate'),
            'center_frequency': LaunchConfiguration('hydra_frequency'),
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_spectrum_analyzer')),
        remappings=[
            ('~/iq_samples', '/hydra_sdr/hydra_sdr_node/iq_samples'),
        ]
    )
    
    # ====================================================================
    # RTL-SDR Node (Optional, if rtlsdr_ros2 package available)
    # ====================================================================
    
    rtlsdr_node = Node(
        package='rtlsdr_ros2',
        executable='rtlsdr_node',
        name='rtlsdr_node',
        namespace='rtlsdr',
        parameters=[{
            'frequency': LaunchConfiguration('rtlsdr_frequency'),
            'sample_rate': 2.4e6,
            'gain': 40.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rtlsdr')),
    )
    
    # ====================================================================
    # Return Launch Description
    # ====================================================================
    
    return LaunchDescription([
        # Device selection
        enable_hydra_arg,
        enable_rtlsdr_arg,
        enable_spectrum_analyzer_arg,
        use_soapy_arg,
        
        # HydraSDR configuration
        hydra_freq_arg,
        hydra_sample_rate_arg,
        hydra_gain_arg,
        lna_gain_arg,
        mixer_gain_arg,
        vga_gain_arg,
        bias_tee_arg,
        sample_type_arg,
        serial_number_arg,
        
        # Spectrum analyzer configuration
        fft_size_arg,
        window_type_arg,
        averaging_arg,
        waterfall_depth_arg,
        peak_threshold_db_arg,
        
        # RTL-SDR configuration
        rtlsdr_freq_arg,
        
        # General options
        auto_stream_arg,
        publish_raw_iq_arg,
        
        # Launch components
        startup_msg,
        hydra_sdr_node,
        spectrum_analyzer_node,
        rtlsdr_node,
    ])

