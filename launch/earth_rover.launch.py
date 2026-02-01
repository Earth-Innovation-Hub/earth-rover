#!/usr/bin/env python3
"""
Earth Rover Unified Launch File

Single configurable launch file for all Earth Rover ROS2 configurations.
Combines vehicle interface, telemetry, sensors, and web services.

Usage:
  # Minimal (MAVROS + Vehicle Interface only)
  ros2 launch earth_rover.launch.py mode:=minimal

  # Vehicle + Telemetry
  ros2 launch earth_rover.launch.py mode:=telemetry

  # Full system with web services
  ros2 launch earth_rover.launch.py mode:=full enable_sdr:=true

  # Custom configuration
  ros2 launch earth_rover.launch.py enable_vehicle:=true enable_telemetry:=true \
      enable_web:=true fcu_url:=/dev/ttyUSB0:57600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all configurable parameters."""
    
    # ========================================================================
    # Launch Mode Argument (simplified interface)
    # ========================================================================
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='custom',
        description='Launch mode: minimal (vehicle only), telemetry (vehicle+telemetry), '
                    'full (all services), custom (use individual enable flags)'
    )
    
    # ========================================================================
    # Vehicle Interface (MAVROS) Arguments
    # ========================================================================
    
    enable_vehicle_arg = DeclareLaunchArgument(
        'enable_vehicle',
        default_value='true',
        description='Enable vehicle interface (MAVROS + vehicle_interface_node)'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600',
        description='FCU connection URL'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://192.168.1.7:14550',
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
    
    # ========================================================================
    # DeepGIS Telemetry Arguments
    # ========================================================================
    
    enable_telemetry_arg = DeclareLaunchArgument(
        'enable_telemetry',
        default_value='false',
        description='Enable DeepGIS telemetry publishing'
    )
    
    deepgis_api_url_arg = DeclareLaunchArgument(
        'deepgis_api_url',
        default_value='http://192.168.0.186:8080',
        description='DeepGIS API URL'
    )
    
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value='',
        description='DeepGIS API key (optional)'
    )
    
    asset_name_arg = DeclareLaunchArgument(
        'asset_name',
        default_value='EarthRover',
        description='Asset/vehicle name for DeepGIS'
    )
    
    session_id_arg = DeclareLaunchArgument(
        'session_id',
        default_value='',
        description='Session ID (auto-generated if empty)'
    )
    
    # ========================================================================
    # Web Services Arguments
    # ========================================================================
    
    enable_web_arg = DeclareLaunchArgument(
        'enable_web',
        default_value='false',
        description='Enable web services (ROSBridge, Web Video Server)'
    )
    
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='ROSBridge WebSocket port'
    )
    
    web_video_port_arg = DeclareLaunchArgument(
        'web_video_port',
        default_value='8080',
        description='Web Video Server port'
    )
    
    # ========================================================================
    # SDR Arguments
    # ========================================================================
    
    enable_sdr_arg = DeclareLaunchArgument(
        'enable_sdr',
        default_value='false',
        description='Enable SDR subsystem (Hydra)'
    )
    
    # ========================================================================
    # Return Launch Description with OpaqueFunction for conditional logic
    # ========================================================================
    
    return LaunchDescription([
        # Mode selection
        mode_arg,
        
        # Vehicle interface arguments
        enable_vehicle_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_namespace_arg,
        
        # Telemetry arguments
        enable_telemetry_arg,
        deepgis_api_url_arg,
        api_key_arg,
        asset_name_arg,
        session_id_arg,
        
        # Web services arguments
        enable_web_arg,
        rosbridge_port_arg,
        web_video_port_arg,
        
        # SDR arguments
        enable_sdr_arg,
        
        # Startup message and node launch
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    """Setup nodes based on configuration with error handling."""
    
    nodes_to_launch = []
    errors = []
    warnings = []
    
    # ========================================================================
    # Parse Mode and Override Enable Flags
    # ========================================================================
    
    try:
        mode = str(context.launch_configurations.get('mode', 'custom'))
    except Exception as e:
        errors.append(f"Failed to parse mode: {e}")
        mode = 'custom'
    
    # Apply mode presets (can be overridden by explicit enable flags)
    if mode == 'minimal':
        enable_vehicle = True
        enable_telemetry = False
        enable_web = False
        enable_sdr = False
    elif mode == 'telemetry':
        enable_vehicle = True
        enable_telemetry = True
        enable_web = False
        enable_sdr = False
    elif mode == 'full':
        enable_vehicle = True
        enable_telemetry = True
        enable_web = True
        enable_sdr = False
    else:  # custom mode
        enable_vehicle = str(context.launch_configurations.get('enable_vehicle', 'true')).lower() == 'true'
        enable_telemetry = str(context.launch_configurations.get('enable_telemetry', 'false')).lower() == 'true'
        enable_web = str(context.launch_configurations.get('enable_web', 'false')).lower() == 'true'
        enable_sdr = str(context.launch_configurations.get('enable_sdr', 'false')).lower() == 'true'
    
    # Allow explicit overrides even in preset modes
    if 'enable_vehicle' in context.launch_configurations:
        enable_vehicle = str(context.launch_configurations['enable_vehicle']).lower() == 'true'
    if 'enable_telemetry' in context.launch_configurations:
        enable_telemetry = str(context.launch_configurations['enable_telemetry']).lower() == 'true'
    if 'enable_web' in context.launch_configurations:
        enable_web = str(context.launch_configurations['enable_web']).lower() == 'true'
    if 'enable_sdr' in context.launch_configurations:
        enable_sdr = str(context.launch_configurations['enable_sdr']).lower() == 'true'
    
    # ========================================================================
    # Vehicle Interface (MAVROS + Vehicle Interface Node)
    # ========================================================================
    
    if enable_vehicle:
        try:
            # Get vehicle parameters with validation
            fcu_url = str(context.launch_configurations.get('fcu_url', 
                '/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTD16B5P-if00-port0:921600'))
            gcs_url = str(context.launch_configurations.get('gcs_url', 'udp://192.168.1.7:14550'))
            
            # Validate URLs
            if not fcu_url:
                warnings.append("FCU URL is empty, MAVROS may fail to connect")
            if not gcs_url:
                warnings.append("GCS URL is empty, ground station link may not work")
            
            try:
                tgt_system = int(context.launch_configurations.get('tgt_system', '1'))
            except (ValueError, TypeError):
                warnings.append(f"Invalid tgt_system value, using default: 1")
                tgt_system = 1
            
            try:
                tgt_component = int(context.launch_configurations.get('tgt_component', '1'))
            except (ValueError, TypeError):
                warnings.append(f"Invalid tgt_component value, using default: 1")
                tgt_component = 1
            
            mavros_namespace = str(context.launch_configurations.get('mavros_namespace', ''))
            
            # MAVROS parameters
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
            
            # MAVROS2 node with error handling
            mavros_node = Node(
                package='mavros',
                executable='mavros_node',
                namespace='mavros',
                parameters=[mavros_params],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[MAVROS] Node exited, will respawn if enabled')
            )
            nodes_to_launch.append(mavros_node)
            
            # Vehicle Interface Node with error handling
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
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[Vehicle Interface] Node exited, will respawn if enabled')
            )
            nodes_to_launch.append(vehicle_interface_node)
            
        except Exception as e:
            errors.append(f"Failed to configure vehicle interface: {e}")
            warnings.append("Vehicle interface nodes will not be launched")
    
    # ========================================================================
    # DeepGIS Telemetry Publisher
    # ========================================================================
    
    if enable_telemetry:
        try:
            deepgis_api_url = str(context.launch_configurations.get('deepgis_api_url', 
                'http://192.168.0.186:8080'))
            api_key = str(context.launch_configurations.get('api_key', ''))
            asset_name = str(context.launch_configurations.get('asset_name', 'EarthRover'))
            session_id = str(context.launch_configurations.get('session_id', ''))
            
            # Validate API URL
            if not deepgis_api_url.startswith(('http://', 'https://')):
                warnings.append(f"DeepGIS API URL may be invalid: {deepgis_api_url}")
            
            if not asset_name:
                warnings.append("Asset name is empty, using default: EarthRover")
                asset_name = 'EarthRover'
            
            telemetry_node = Node(
                package='deepgis_vehicles',
                executable='deepgis_telemetry_publisher.py',
                name='deepgis_telemetry_publisher',
                output='screen',
                respawn=True,
                respawn_delay=3.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[Telemetry] Node exited, will respawn if enabled'),
                parameters=[{
                    'deepgis_api_url': deepgis_api_url,
                    'api_key': api_key,
                    'asset_name': asset_name,
                    'session_id': session_id,
                    'project_title': 'Earth Rover Data Collection',
                    'flight_mode': 'AUTO',
                    'mission_type': 'Telemetry Collection',
                    'mavros_namespace': '/mavros',
                    'publish_rate': 1.0,
                    'batch_size': 10,
                    'enable_batch_mode': True,
                }]
            )
            nodes_to_launch.append(telemetry_node)
            
        except Exception as e:
            errors.append(f"Failed to configure telemetry: {e}")
            warnings.append("Telemetry node will not be launched")
    
    # ========================================================================
    # Web Services
    # ========================================================================
    
    if enable_web:
        try:
            rosbridge_port_str = str(context.launch_configurations.get('rosbridge_port', '9090'))
            web_video_port_str = str(context.launch_configurations.get('web_video_port', '8080'))
            
            # Validate and parse ports
            try:
                rosbridge_port = int(rosbridge_port_str)
                if not (1024 <= rosbridge_port <= 65535):
                    warnings.append(f"ROSBridge port {rosbridge_port} is outside recommended range (1024-65535)")
            except (ValueError, TypeError):
                errors.append(f"Invalid ROSBridge port: {rosbridge_port_str}, using default: 9090")
                rosbridge_port = 9090
            
            try:
                web_video_port = int(web_video_port_str)
                if not (1024 <= web_video_port <= 65535):
                    warnings.append(f"Web Video port {web_video_port} is outside recommended range (1024-65535)")
            except (ValueError, TypeError):
                errors.append(f"Invalid Web Video port: {web_video_port_str}, using default: 8080")
                web_video_port = 8080
            
            # Check for port conflicts
            if rosbridge_port == web_video_port:
                errors.append(f"Port conflict: ROSBridge and Web Video Server both using port {rosbridge_port}")
                warnings.append("Web services may not work correctly due to port conflict")
            
            # ROSBridge WebSocket with error handling
            rosbridge_node = Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                parameters=[{
                    'port': rosbridge_port,
                    'address': '0.0.0.0',
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[ROSBridge] Node exited, will respawn if enabled')
            )
            nodes_to_launch.append(rosbridge_node)
            
            # Web Video Server with error handling
            web_video_node = Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video_server',
                parameters=[{
                    'port': web_video_port,
                    'address': '0.0.0.0',
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[Web Video Server] Node exited, will respawn if enabled')
            )
            nodes_to_launch.append(web_video_node)
            
        except Exception as e:
            errors.append(f"Failed to configure web services: {e}")
            warnings.append("Web service nodes will not be launched")
    
    # ========================================================================
    # SDR Subsystem
    # ========================================================================
    
    if enable_sdr:
        try:
            hydra_sdr_node = Node(
                package='deepgis_vehicles',
                executable='hydra_sdr_node.py',
                name='hydra_sdr_node',
                namespace='hydra_sdr',
                parameters=[{
                    'auto_connect': True,
                    'auto_stream': True,  # Auto-start streaming when SDR is enabled
                    'center_frequency': 433.0e6,
                    'sample_rate': 2.5e6,
                }],
                output='screen',
                respawn=True,
                respawn_delay=3.0,
                emulate_tty=True,
                on_exit=LogInfo(msg='[Hydra SDR] Node exited, will respawn if enabled')
            )
            nodes_to_launch.append(hydra_sdr_node)
            
        except Exception as e:
            errors.append(f"Failed to configure SDR subsystem: {e}")
            warnings.append("SDR node will not be launched")
    
    # ========================================================================
    # Print startup message and error/warning summary
    # ========================================================================
    
    services_list = []
    if enable_vehicle:
        services_list.append("MAVROS + Vehicle Interface")
    if enable_telemetry:
        services_list.append("DeepGIS Telemetry")
    if enable_web:
        services_list.append("ROSBridge + Web Video Server")
    if enable_sdr:
        services_list.append("Hydra SDR")
    
    services_str = ", ".join(services_list) if services_list else "None"
    
    print("╔════════════════════════════════════════════════════════════╗")
    print("║           Earth Rover - Unified Launch File               ║")
    print("╚════════════════════════════════════════════════════════════╝")
    print(f"  Mode:     {mode}")
    print(f"  Services: {services_str}")
    print(f"  Nodes:    {len(nodes_to_launch)} configured")
    
    # Print warnings
    if warnings:
        print("\n  ⚠ Warnings:")
        for warning in warnings:
            print(f"    - {warning}")
    
    # Print errors (non-fatal, launch continues)
    if errors:
        print("\n  ⚠ Errors (launch will continue):")
        for error in errors:
            print(f"    - {error}")
        print("  Note: Some nodes may not start due to configuration errors.")
    
    print("")
    
    # Log warnings and errors as launch actions
    launch_actions = []
    if warnings:
        for warning in warnings:
            launch_actions.append(LogInfo(msg=f'[WARNING] {warning}'))
    if errors:
        for error in errors:
            launch_actions.append(LogInfo(msg=f'[ERROR] {error}'))
    
    return nodes_to_launch + launch_actions

