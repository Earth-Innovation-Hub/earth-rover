"""
Launch file for GPS Reader with Map Visualization
Launches GPS reader and map visualizer together.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for GPS reader with map visualization."""

    # GPS L1 frequency: 1575.42 MHz
    GPS_L1_FREQUENCY = 1575.42e6

    # GPS Reader arguments
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
        description='Sample rate in Hz'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='auto',
        description='Gain setting'
    )

    enable_position_decoding_arg = DeclareLaunchArgument(
        'enable_position_decoding',
        default_value='true',
        description='Enable GPS position decoding'
    )

    # Map visualizer arguments
    map_port_arg = DeclareLaunchArgument(
        'map_port',
        default_value='8080',
        description='Port for web map interface'
    )

    map_host_arg = DeclareLaunchArgument(
        'map_host',
        default_value='0.0.0.0',
        description='Host for web map interface (0.0.0.0 for all interfaces)'
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
            'enable_position_decoding': LaunchConfiguration('enable_position_decoding'),
        }],
    )

    # Map visualizer node
    map_visualizer_node = Node(
        package='rtlsdr_ros2',
        executable='gps_map_visualizer',
        name='gps_map_visualizer',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('map_port'),
            'host': LaunchConfiguration('map_host'),
        }],
    )

    return LaunchDescription([
        device_index_arg,
        center_frequency_arg,
        sample_rate_arg,
        gain_arg,
        enable_position_decoding_arg,
        map_port_arg,
        map_host_arg,
        gps_reader_node,
        map_visualizer_node,
    ])

