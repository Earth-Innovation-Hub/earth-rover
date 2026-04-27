#!/usr/bin/env python3
"""
GPS Position Publisher Node for RTL-SDR ROS2 Package
Subscribes to GPS data and publishes actual GPS position using NavSatFix messages.
This node can use external GPS decoders or a built-in simplified decoder.
"""

import sys
import os

# Add package path for imports when running as script
_script_dir = os.path.dirname(os.path.abspath(__file__))
# Try to find the site-packages directory
_package_base = os.path.dirname(os.path.dirname(_script_dir))  # Go up from lib/rtlsdr_ros2 to install/rtlsdr_ros2
_site_packages = os.path.join(_package_base, 'lib', 'python3.10', 'site-packages')
if os.path.exists(_site_packages) and _site_packages not in sys.path:
    sys.path.insert(0, _site_packages)
# Also add the script directory as fallback
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
from rtlsdr_ros2.msg import GpsData
try:
    from .gps_decoder import create_gps_decoder, SimpleGpsDecoder
except (ImportError, ValueError):
    # Try absolute import
    try:
        from rtlsdr_ros2.gps_decoder import create_gps_decoder, SimpleGpsDecoder
    except ImportError:
        # Last resort: direct import from site-packages directory
        import importlib.util
        _gps_decoder_path = os.path.join(_site_packages, "rtlsdr_ros2", "gps_decoder.py")
        if os.path.exists(_gps_decoder_path):
            spec = importlib.util.spec_from_file_location("gps_decoder", _gps_decoder_path)
            gps_decoder = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(gps_decoder)
            create_gps_decoder = gps_decoder.create_gps_decoder
            SimpleGpsDecoder = gps_decoder.SimpleGpsDecoder
        else:
            raise ImportError(f"Could not find gps_decoder module at {_gps_decoder_path}")

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


class GpsPositionPublisherNode(Node):
    """ROS2 Node that decodes GPS signals and publishes position fixes."""

    def __init__(self):
        super().__init__('gps_position_publisher')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_external_decoder', False, ParameterDescriptor(description='Use external GPS decoder')),
                ('external_decoder_path', '', ParameterDescriptor(description='Path to external decoder module')),
                ('min_snr_for_fix', 5.0, ParameterDescriptor(description='Minimum SNR (dB) required for position fix')),
                ('publish_rate', 1.0, ParameterDescriptor(description='Position publishing rate in Hz')),
            ]
        )

        # Get parameters
        self.use_external_decoder = self.get_parameter('use_external_decoder').value
        self.external_decoder_path = self.get_parameter('external_decoder_path').value
        self.min_snr_for_fix = self.get_parameter('min_snr_for_fix').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize GPS decoder
        self.decoder = None
        self.sample_rate = 2.048e6  # Will be updated from messages
        self.init_decoder()

        # Create subscriber for GPS data
        self.gps_data_subscriber = self.create_subscription(
            GpsData,
            'rtlsdr/gps_data',
            self.gps_data_callback,
            10
        )

        # Create publisher for GPS position
        self.position_publisher = self.create_publisher(
            NavSatFix,
            'rtlsdr/gps_position',
            10
        )

        # State tracking
        self.last_samples = None
        self.samples_buffer = []
        self.buffer_size = 10  # Buffer samples for processing

        # Create timer for periodic position publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('GPS Position Publisher Node initialized')
        self.get_logger().info(f'Using external decoder: {self.use_external_decoder}')

    def init_decoder(self):
        """Initialize GPS decoder."""
        try:
            if self.use_external_decoder and self.external_decoder_path:
                # Load external decoder
                import importlib.util
                spec = importlib.util.spec_from_file_location("external_gps_decoder", self.external_decoder_path)
                decoder_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(decoder_module)
                
                # Get decoder function
                if hasattr(decoder_module, 'decode_gps'):
                    decoder_func = decoder_module.decode_gps
                    self.decoder = SimpleGpsDecoder(self.sample_rate)
                    self.decoder.set_external_decoder(decoder_func)
                    self.get_logger().info(f'Loaded external decoder from: {self.external_decoder_path}')
                else:
                    self.get_logger().warn('External decoder module missing decode_gps function')
                    self.decoder = create_gps_decoder(self.sample_rate, use_simple=True)
            else:
                # Use built-in decoder
                self.decoder = create_gps_decoder(self.sample_rate, use_simple=False)
                self.get_logger().info('Using built-in GPS decoder')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPS decoder: {str(e)}')
            self.decoder = create_gps_decoder(self.sample_rate, use_simple=True)

    def gps_data_callback(self, msg: GpsData):
        """Process incoming GPS data messages."""
        # Update sample rate if changed
        if abs(self.sample_rate - msg.sample_rate) > 1000:
            self.sample_rate = msg.sample_rate
            if self.decoder:
                self.decoder.sample_rate = self.sample_rate

        # Check signal quality
        if msg.snr_db < self.min_snr_for_fix:
            return  # Signal too weak

        # Convert IQ samples to complex array
        i_samples = np.array(msg.i_samples, dtype=np.float32)
        q_samples = np.array(msg.q_samples, dtype=np.float32)
        samples = i_samples + 1j * q_samples

        # Store samples for processing
        self.samples_buffer.append(samples)
        if len(self.samples_buffer) > self.buffer_size:
            self.samples_buffer.pop(0)

        # Store last samples for timer callback
        self.last_samples = samples
        self.last_msg = msg

    def timer_callback(self):
        """Periodically process samples and publish position."""
        if self.decoder is None or self.last_samples is None:
            return

        try:
            # Process samples with decoder
            # Use concatenated buffer for better processing
            if len(self.samples_buffer) > 0:
                combined_samples = np.concatenate(self.samples_buffer)
            else:
                combined_samples = self.last_samples

            # Decode GPS position
            position_data = self.decoder.process_samples(combined_samples)

            if position_data:
                # Publish position fix
                self.publish_position(position_data)
            else:
                # No fix available yet
                self.get_logger().debug('Waiting for GPS fix...')

        except Exception as e:
            self.get_logger().error(f'Error processing GPS samples: {str(e)}')

    def publish_position(self, position_data: dict):
        """Publish GPS position as NavSatFix message."""
        msg = NavSatFix()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        # Status
        msg.status.status = NavSatStatus.STATUS_FIX if position_data.get('fix_type', 0) >= 2 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position
        msg.latitude = position_data.get('latitude', 0.0)
        msg.longitude = position_data.get('longitude', 0.0)
        msg.altitude = position_data.get('altitude', 0.0)

        # Position covariance (3x3 matrix flattened to 9 elements)
        covariance = position_data.get('position_covariance', [0.0] * 9)
        if len(covariance) == 9:
            msg.position_covariance = covariance
        else:
            # Default covariance based on HDOP
            hdop = position_data.get('hdop', 5.0)
            variance = (hdop * 5.0) ** 2  # Rough conversion
            msg.position_covariance = [
                variance, 0.0, 0.0,
                0.0, variance, 0.0,
                0.0, 0.0, variance * 4.0  # Vertical typically worse
            ]

        msg.position_covariance_type = position_data.get('position_covariance_type', NavSatFix.COVARIANCE_TYPE_UNKNOWN)

        self.position_publisher.publish(msg)

        # Log position
        self.get_logger().info(
            f'GPS Position Fix: '
            f'Lat: {msg.latitude:.6f}°, '
            f'Lon: {msg.longitude:.6f}°, '
            f'Alt: {msg.altitude:.1f}m, '
            f'HDOP: {position_data.get("hdop", 0.0):.2f}, '
            f'Satellites: {position_data.get("satellites_used", 0)}'
        )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        node = GpsPositionPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

