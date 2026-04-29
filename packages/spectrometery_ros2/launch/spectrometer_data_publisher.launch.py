"""Launch spectrometer publisher and optional Intensity_Plot (spectrum image + dip markers)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topic',
            default_value='spectrometer',
            description='Float64MultiArray topic for spectrum payloads'),
        DeclareLaunchArgument(
            'integration_time_micros',
            default_value='500000',
            description='Spectrometer integration time in microseconds'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='0.1',
            description='Timer period between acquisitions (seconds)'),
        DeclareLaunchArgument(
            'plot_image',
            default_value='true',
            description=(
                'If true, also launch Intensity_Plot.py (subscribes to topic, publishes plot image). '
                "Set to 'false' for hardware-only publishing.")),
        DeclareLaunchArgument(
            'dip_prominence',
            default_value='0.05',
            description='Intensity_Plot --dip-prominence (min dip depth vs spectrum max)'),
        DeclareLaunchArgument(
            'dip_min_distance',
            default_value='15',
            description='Intensity_Plot --dip-min-distance (min index spacing between dips)'),
        DeclareLaunchArgument(
            'plot_image_topic',
            default_value='spectrometer_plot',
            description='sensor_msgs/Image topic published by Intensity_Plot'),
        Node(
            package='spectrometery_ros2',
            executable='Spectrometer_Data_Publisher.py',
            name='spectrometer_data_publisher',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('topic'),
                'integration_time_micros': ParameterValue(
                    LaunchConfiguration('integration_time_micros'), value_type=int),
                'publish_period_sec': ParameterValue(
                    LaunchConfiguration('publish_period_sec'), value_type=float),
            }],
        ),
        Node(
            package='spectrometery_ros2',
            executable='Intensity_Plot.py',
            name='spectrometer_intensity_plot',
            output='screen',
            arguments=[
                '--topic', LaunchConfiguration('topic'),
                '--dip-prominence', LaunchConfiguration('dip_prominence'),
                '--dip-min-distance', LaunchConfiguration('dip_min_distance'),
                '--image-topic', LaunchConfiguration('plot_image_topic'),
            ],
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('plot_image'), "' == 'true'"])),
        ),
    ])
