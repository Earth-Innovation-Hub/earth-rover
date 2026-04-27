"""
Launch file for RTL-SDR ROS2 node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RTL-SDR reader node."""

    # Declare launch arguments
    device_index_arg = DeclareLaunchArgument(
        'device_index',
        default_value='0',
        description='RTL-SDR device index'
    )

    center_frequency_arg = DeclareLaunchArgument(
        'center_frequency',
        default_value='100.0e6',
        description='Center frequency in Hz (default: 100 MHz)'
    )

    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='2.048e6',
        description='Sample rate in Hz (default: 2.048 MSps)'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='auto',
        description='Gain setting (auto or value in dB)'
    )

    num_samples_arg = DeclareLaunchArgument(
        'num_samples',
        default_value='1024',
        description='Number of samples per read'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )

    publish_raw_iq_arg = DeclareLaunchArgument(
        'publish_raw_iq',
        default_value='true',
        description='Publish raw IQ samples'
    )

    publish_spectrum_arg = DeclareLaunchArgument(
        'publish_spectrum',
        default_value='true',
        description='Publish spectrum data'
    )

    fft_size_arg = DeclareLaunchArgument(
        'fft_size',
        default_value='1024',
        description='FFT size for spectrum analysis'
    )

    use_config_arg = DeclareLaunchArgument(
        'use_config',
        default_value='false',
        description='Use configuration file instead of launch arguments'
    )

    # Get config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('rtlsdr_ros2'),
        'config',
        'rtlsdr_config.yaml'
    ])

    # RTL-SDR reader node
    rtlsdr_node = Node(
        package='rtlsdr_ros2',
        executable='rtlsdr_reader',
        name='rtlsdr_reader',
        output='screen',
        parameters=[{
            'device_index': LaunchConfiguration('device_index'),
            'center_frequency': LaunchConfiguration('center_frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'gain': LaunchConfiguration('gain'),
            'num_samples': LaunchConfiguration('num_samples'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'publish_raw_iq': LaunchConfiguration('publish_raw_iq'),
            'publish_spectrum': LaunchConfiguration('publish_spectrum'),
            'fft_size': LaunchConfiguration('fft_size'),
        }],
        remappings=[
            ('rtlsdr/signal', 'rtlsdr/signal'),
            ('rtlsdr/spectrum', 'rtlsdr/spectrum'),
        ]
    )

    return LaunchDescription([
        device_index_arg,
        center_frequency_arg,
        sample_rate_arg,
        gain_arg,
        num_samples_arg,
        publish_rate_arg,
        publish_raw_iq_arg,
        publish_spectrum_arg,
        fft_size_arg,
        use_config_arg,
        rtlsdr_node,
    ])

