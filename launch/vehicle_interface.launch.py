from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0:921600',
        description='FCU connection URL (e.g., udp://:14540@127.0.0.1:14557 for SITL, '
                    '/dev/ttyUSB0:57600 for serial, or /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0:921600 for TTL232)'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@127.0.0.1:14550',
        description='GCS connection URL'
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    
    tgt_component_arg = DeclareLaunchArgument(
        'tgt_component',
        default_value='1',
        description='Target component ID'
    )
    
    mavros_namespace_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='',
        description='MAVROS namespace (empty for default /mavros)'
    )
    
    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_namespace_arg,
        LogInfo(msg=['Starting MAVROS2 with FCU URL: ', LaunchConfiguration('fcu_url')]),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    # Get launch configuration values with defaults - ensure they're strings
    fcu_url = str(context.launch_configurations.get('fcu_url', '/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16AXT-if00-port0:921600'))
    gcs_url = str(context.launch_configurations.get('gcs_url', 'udp://@127.0.0.1:14550'))
    tgt_system = int(context.launch_configurations.get('tgt_system', '1'))
    tgt_component = int(context.launch_configurations.get('tgt_component', '1'))
    mavros_namespace = str(context.launch_configurations.get('mavros_namespace', ''))
    
    # MAVROS2 uses /mavros as default namespace, so don't set namespace on the node
    # The namespace parameter in the node configures the UAS prefix internally
    namespace_for_node = mavros_namespace if mavros_namespace else None
    
    # Build parameters dictionary
    mavros_params = {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'target_system_id': tgt_system,
        'target_component_id': tgt_component,
        'fcu_protocol': 'v2.0',
        'system_id': 1,
        'component_id': 240,
        'connection_timeout': 10.0,
        'startup_px4_usb_quirk': True,
        'diagnostic_period': 1.0,
    }
    
    # MAVROS2 node - use namespace to avoid service name collisions
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        parameters=[mavros_params],
        output='screen'
    )
    
    # Vehicle Interface Node parameters - use /mavros as default
    mavros_ns_for_params = mavros_namespace if mavros_namespace else '/mavros'
    vehicle_params = {
        'mavros_namespace': mavros_ns_for_params,
        'fcu_url': fcu_url,
        'connection_timeout': 10.0,
    }
    
    vehicle_interface_node = Node(
        package='deepgis_vehicles',
        executable='vehicle_interface_node',
        name='vehicle_interface_node',
        parameters=[vehicle_params],
        output='screen'
    )
    
    return [mavros_node, vehicle_interface_node]

