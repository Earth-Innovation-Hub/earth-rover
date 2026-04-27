"""
Launch file for GPS Reader Node using RTL-SDR
Tuned to GPS L1 frequency (1575.42 MHz)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GPS reader node."""

    # GPS L1 frequency: 1575.42 MHz
    GPS_L1_FREQUENCY = 1575.42e6

    # Declare launch arguments
    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='RTL-SDR device index'
    )

    center_frequency_arg = DeclareLaunchArgument(
        'center_frequency',
        default_value=str(GPS_L1_FREQUENCY),
        description='Center frequency in Hz (default: GPS L1 at 1575.42 MHz)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='2.048e6',
        description='Sample rate in Hz (default: 2.048 MSps)'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='auto',
        description='Gain setting (auto or value in dB, higher gain recommended for GPS)'
    )

    num_samples_arg = DeclareLaunchArgument(
        'num_samples',
        default_value='16384',
        description='Number of samples per read (larger values recommended for GPS)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='5.0',
        description='Publishing rate in Hz'
    )

    enable_signal_analysis_arg = DeclareLaunchArgument(
        'enable_signal_analysis',
        default_value='true',
        description='Enable GPS signal quality analysis'
    )

    enable_position_decoding_arg = DeclareLaunchArgument(
        'enable_position_decoding',
        default_value='true',
        description='Enable GPS position decoding and publishing'
    )

    min_snr_for_fix_arg = DeclareLaunchArgument(
        'min_snr_for_fix',
        default_value='5.0',
        description='Minimum SNR (dB) required for position fix'
    )

    # GPS reader node
    gps_reader_node = Node(
        package='rtlsdr_ros2',
        executable='gps_reader',
        name='gps_reader',
        output='screen',
        parameters=[{
            'device_index': LaunchConfiguration('device_index'),
            'center_frequency': LaunchConfiguration('center_frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'gain': LaunchConfiguration('gain'),
            'num_samples': LaunchConfiguration('num_samples'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'enable_signal_analysis': LaunchConfiguration('enable_signal_analysis'),
            'enable_position_decoding': LaunchConfiguration('enable_position_decoding'),
            'min_snr_for_fix': LaunchConfiguration('min_snr_for_fix'),
        }],
        remappings=[
            ('rtlsdr/gps_data', 'rtlsdr/gps_data'),
        ]
    )

    return LaunchDescription([
        device_index_arg,
        center_frequency_arg,
        sample_rate_arg,
        gain_arg,
        num_samples_arg,
        publish_rate_arg,
        enable_signal_analysis_arg,
        enable_position_decoding_arg,
        min_snr_for_fix_arg,
        gps_reader_node,
    ])

