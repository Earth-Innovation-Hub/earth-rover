#!/usr/bin/env python3
"""
Earth Rover RTL-SDR V4 Subsystem Launch

Launches SDR-related nodes for RTL-SDR Blog V4 integration:
- RTL-SDR V4 driver node (using rtl_sdr tools)
- Spectrum analyzer with waterfall and peak detection (shared node)
- Optional: SDR visualizer (PyQt5 GUI)

Hardware Support:
- RTL-SDR Blog V4 (R828D tuner, RTL2832U, 24-1766 MHz)
- Sample rates: 225 kHz - 3.2 MSPS (2.4 MSPS recommended)
- Gain: auto or 0-49.6 dB (discrete tuner steps)
- Features: Bias-T 4.5V, direct sampling for HF, PPM correction

Usage Examples:
  # Default -- 433 MHz ISM band, auto-gain, 2.4 MSPS
  ros2 launch deepgis_vehicles rtl_sdr.launch.py

  # Custom frequency and gain
  ros2 launch deepgis_vehicles rtl_sdr.launch.py frequency:=915.0e6 gain:=28.0

  # Enable bias tee for active antenna
  ros2 launch deepgis_vehicles rtl_sdr.launch.py bias_tee:=true

  # HF direct sampling (500 kHz - 28.8 MHz)
  ros2 launch deepgis_vehicles rtl_sdr.launch.py frequency:=7.0e6 direct_sampling:=2

  # With visualizer disabled (headless)
  ros2 launch deepgis_vehicles rtl_sdr.launch.py enable_visualizer:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ====================================================================
    # Launch Arguments -- Device Configuration
    # ====================================================================

    freq_arg = DeclareLaunchArgument(
        'frequency',
        default_value='433.0e6',
        description='Center frequency in Hz (24 MHz - 1766 MHz)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='2.4e6',
        description='Sample rate in Hz (225 kHz - 3.2 MSPS, 2.4 recommended)'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='-1.0',
        description='Tuner gain in dB (-1 for auto, 0-49.6 for manual)'
    )

    bias_tee_arg = DeclareLaunchArgument(
        'bias_tee',
        default_value='false',
        description='Enable 4.5V bias tee for active antenna/LNA'
    )

    ppm_arg = DeclareLaunchArgument(
        'ppm_correction',
        default_value='0',
        description='Frequency correction in PPM'
    )

    direct_sampling_arg = DeclareLaunchArgument(
        'direct_sampling',
        default_value='0',
        description='Direct sampling: 0=off, 1=I-branch (HF), 2=Q-branch (HF)'
    )

    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='RTL-SDR device index for multi-dongle setups'
    )

    agc_arg = DeclareLaunchArgument(
        'agc_mode',
        default_value='false',
        description='Enable RTL2832U internal AGC'
    )

    # ====================================================================
    # Launch Arguments -- Spectrum Analyzer
    # ====================================================================

    enable_spectrum_arg = DeclareLaunchArgument(
        'enable_spectrum_analyzer',
        default_value='true',
        description='Enable spectrum analyzer node'
    )

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

    peak_threshold_arg = DeclareLaunchArgument(
        'peak_threshold_db',
        default_value='10.0',
        description='Peak detection threshold above noise floor (dB)'
    )

    # ====================================================================
    # Launch Arguments -- Visualizer
    # ====================================================================

    enable_visualizer_arg = DeclareLaunchArgument(
        'enable_visualizer',
        default_value='false',
        description='Enable SDR visualizer GUI (requires PyQt5)'
    )

    # ====================================================================
    # Launch Arguments -- General
    # ====================================================================

    auto_stream_arg = DeclareLaunchArgument(
        'auto_stream',
        default_value='true',
        description='Automatically start streaming on launch'
    )

    publish_raw_iq_arg = DeclareLaunchArgument(
        'publish_raw_iq',
        default_value='true',
        description='Publish raw IQ samples (high bandwidth)'
    )

    # ====================================================================
    # Startup Banner
    # ====================================================================

    startup_msg = LogInfo(msg="""
╔════════════════════════════════════════════════════════════╗
║         Earth Rover - RTL-SDR V4 Subsystem                 ║
╠════════════════════════════════════════════════════════════╣
║  Hardware: RTL-SDR Blog V4 (RTL2832U + R828D)              ║
║  Frequency Range: 24-1766 MHz (HF via direct sampling)     ║
║  Sample Rate: 225 kHz - 3.2 MSPS (2.4 recommended)        ║
║  Gain: Auto or 0-49.6 dB                                   ║
║  Features: Bias-T, PPM correction, HF direct sampling      ║
╚════════════════════════════════════════════════════════════╝
    """)

    # ====================================================================
    # RTL-SDR V4 Driver Node
    # ====================================================================

    rtl_sdr_node = Node(
        package='deepgis_vehicles',
        executable='rtl_sdr_node.py',
        name='rtl_sdr_node',
        namespace='rtl_sdr',
        parameters=[{
            'auto_connect': True,
            'auto_stream': LaunchConfiguration('auto_stream'),
            'center_frequency': LaunchConfiguration('frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'gain': LaunchConfiguration('gain'),
            'bias_tee': LaunchConfiguration('bias_tee'),
            'ppm_correction': LaunchConfiguration('ppm_correction'),
            'direct_sampling': LaunchConfiguration('direct_sampling'),
            'device_index': LaunchConfiguration('device_index'),
            'agc_mode': LaunchConfiguration('agc_mode'),
            'publish_raw_iq': LaunchConfiguration('publish_raw_iq'),
            'publish_spectrum': True,
            'fft_size': LaunchConfiguration('fft_size'),
            'spectrum_averaging': LaunchConfiguration('averaging'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # ====================================================================
    # Spectrum Analyzer Node (shared with HydraSDR -- topic remapped)
    # ====================================================================

    spectrum_analyzer_node = Node(
        package='deepgis_vehicles',
        executable='spectrum_analyzer_node.py',
        name='spectrum_analyzer_node',
        namespace='rtl_sdr',
        parameters=[{
            'fft_size': LaunchConfiguration('fft_size'),
            'window_type': LaunchConfiguration('window_type'),
            'averaging': LaunchConfiguration('averaging'),
            'waterfall_depth': LaunchConfiguration('waterfall_depth'),
            'peak_threshold_db': LaunchConfiguration('peak_threshold_db'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'center_frequency': LaunchConfiguration('frequency'),
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_spectrum_analyzer')),
        remappings=[
            ('~/iq_samples', '/rtl_sdr/rtl_sdr_node/iq_samples'),
        ]
    )

    # ====================================================================
    # SDR Visualizer Node (shared -- topic remapped)
    # ====================================================================

    sdr_visualizer_node = Node(
        package='deepgis_vehicles',
        executable='sdr_visualizer.py',
        name='sdr_visualizer',
        namespace='rtl_sdr_viz',
        parameters=[{
            'spectrum_topic': '/rtl_sdr/rtl_sdr_node/spectrum',
            'waterfall_topic': '/rtl_sdr/spectrum_analyzer_node/waterfall',
            'iq_topic': '/rtl_sdr/rtl_sdr_node/iq_samples',
            'center_frequency': LaunchConfiguration('frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
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
        # Device configuration
        freq_arg,
        sample_rate_arg,
        gain_arg,
        bias_tee_arg,
        ppm_arg,
        direct_sampling_arg,
        device_index_arg,
        agc_arg,

        # Spectrum analyzer
        enable_spectrum_arg,
        fft_size_arg,
        window_type_arg,
        averaging_arg,
        waterfall_depth_arg,
        peak_threshold_arg,

        # Visualizer
        enable_visualizer_arg,

        # General
        auto_stream_arg,
        publish_raw_iq_arg,

        # Nodes
        startup_msg,
        rtl_sdr_node,
        spectrum_analyzer_node,
        sdr_visualizer_node,
    ])
