from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_device',
            default_value='',
            description='Serial device path; empty uses laser_ranger serial_device_default param'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('topic_raw', default_value='serial_data'),
        DeclareLaunchArgument('topic_distance', default_value='laser_distance'),
        Node(
            package='laser_ranger',
            executable='laser_ranger_node',
            name='laser_ranger',
            output='screen',
            parameters=[{
                'serial_device': LaunchConfiguration('serial_device'),
                'baud_rate': ParameterValue(
                    LaunchConfiguration('baud_rate'), value_type=int),
                'topic_raw': LaunchConfiguration('topic_raw'),
                'topic_distance': LaunchConfiguration('topic_distance'),
            }],
        ),
    ])
