#!/usr/bin/env python3
"""
GPS Reader Node for RTL-SDR ROS2 Package
This node specifically tunes to GPS L1 frequency (1575.42 MHz) and captures GPS signals.
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
from rtlsdr import RtlSdr
import time

from rtlsdr_ros2.msg import GpsData
try:
    from .gps_decoder import create_gps_decoder
except (ImportError, ValueError):
    # Try absolute import
    try:
        from rtlsdr_ros2.gps_decoder import create_gps_decoder
    except ImportError:
        # Last resort: direct import from site-packages directory
        import importlib.util
        _gps_decoder_path = os.path.join(_site_packages, "rtlsdr_ros2", "gps_decoder.py")
        if os.path.exists(_gps_decoder_path):
            spec = importlib.util.spec_from_file_location("gps_decoder", _gps_decoder_path)
            gps_decoder = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(gps_decoder)
            create_gps_decoder = gps_decoder.create_gps_decoder
        else:
            raise ImportError(f"Could not find gps_decoder module at {_gps_decoder_path}")

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


class GpsReaderNode(Node):
    """ROS2 Node for reading GPS signals from RTL-SDR device."""

    def __init__(self):
        super().__init__('gps_reader')

        # GPS L1 frequency: 1575.42 MHz
        GPS_L1_FREQUENCY = 1575.42e6

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_index', 0, ParameterDescriptor(description='RTL-SDR device index')),
                ('center_frequency', GPS_L1_FREQUENCY, ParameterDescriptor(description='Center frequency in Hz (default: GPS L1)')),
                ('sample_rate', 2.048e6, ParameterDescriptor(description='Sample rate in Hz')),
                ('gain', 'auto', ParameterDescriptor(description='Gain setting (auto or value in dB)')),
                ('num_samples', 16384, ParameterDescriptor(description='Number of samples per read (larger for GPS)')),
                ('publish_rate', 5.0, ParameterDescriptor(description='Publishing rate in Hz')),
                ('enable_signal_analysis', True, ParameterDescriptor(description='Enable signal quality analysis')),
                ('enable_position_decoding', True, ParameterDescriptor(description='Enable GPS position decoding')),
                ('min_snr_for_fix', 5.0, ParameterDescriptor(description='Minimum SNR (dB) required for position fix')),
            ]
        )

        # Get parameters
        self.device_index = self.get_parameter('device_index').value
        self.center_frequency = self.get_parameter('center_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.gain_setting = self.get_parameter('gain').value
        self.num_samples = self.get_parameter('num_samples').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_signal_analysis = self.get_parameter('enable_signal_analysis').value
        self.enable_position_decoding = self.get_parameter('enable_position_decoding').value
        self.min_snr_for_fix = self.get_parameter('min_snr_for_fix').value

        # Create publishers
        self.gps_publisher = self.create_publisher(
            GpsData,
            'rtlsdr/gps_data',
            10
        )

        # Position publisher (if decoding enabled)
        if self.enable_position_decoding:
            self.position_publisher = self.create_publisher(
                NavSatFix,
                'rtlsdr/gps_position',
                10
            )
            # Initialize GPS decoder
            self.gps_decoder = create_gps_decoder(self.sample_rate, use_simple=False)
            self.samples_buffer = []
            self.buffer_size = 10
            self.get_logger().info('GPS position decoding enabled')
        else:
            self.gps_decoder = None

        # Initialize RTL-SDR device
        self.sdr = None
        self.serial_number = "unknown"
        self.is_running = False
        self.sample_counter = 0
        self.init_device()

        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('GPS Reader Node initialized')
        self.get_logger().info(f'Tuned to GPS L1: {self.center_frequency/1e6:.3f} MHz')

    def init_device(self):
        """Initialize the RTL-SDR device for GPS reception."""
        try:
            self.sdr = RtlSdr(self.device_index)

            # Configure device for GPS L1
            self.sdr.sample_rate = self.sample_rate
            self.sdr.center_freq = self.center_frequency

            # Set gain - GPS signals are weak, so higher gain may be needed
            if self.gain_setting == 'auto':
                self.sdr.gain = 'auto'
                self.get_logger().info('Gain set to auto')
            else:
                self.sdr.gain = float(self.gain_setting)
                self.get_logger().info(f'Gain set to {self.gain_setting} dB')

            # Get device info
            try:
                self.serial_number = str(self.device_index)
            except Exception:
                self.serial_number = f"device_{self.device_index}"

            self.is_running = True

            self.get_logger().info(
                f'RTL-SDR device initialized for GPS:\n'
                f'  Device Index: {self.device_index}\n'
                f'  Center Frequency: {self.center_frequency/1e6:.6f} MHz (GPS L1)\n'
                f'  Sample Rate: {self.sample_rate/1e6:.3f} MSps\n'
                f'  Gain: {self.gain_setting}\n'
                f'  Samples per read: {self.num_samples}\n'
                f'  Serial Number: {self.serial_number}'
            )

            # GPS-specific warnings
            self.get_logger().warn(
                'NOTE: GPS signals are very weak and require:\n'
                '  - Good outdoor antenna (GPS patch antenna recommended)\n'
                '  - Clear view of sky\n'
                '  - Proper antenna placement and orientation'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to initialize RTL-SDR device: {str(e)}')
            self.get_logger().error('Please ensure RTL-SDR device is connected and drivers are installed.')
            self.is_running = False

    def timer_callback(self):
        """Timer callback to read and publish GPS data."""
        if not self.is_running or self.sdr is None:
            return

        try:
            # Read IQ samples from RTL-SDR
            samples = self.sdr.read_samples(self.num_samples)

            # Publish GPS data
            self.publish_gps_data(samples)

            # Decode and publish position if enabled
            if self.enable_position_decoding and self.gps_decoder:
                self.process_position_decoding(samples)

        except Exception as e:
            self.get_logger().error(f'Error reading GPS data from RTL-SDR: {str(e)}')

    def analyze_signal_quality(self, samples):
        """Analyze GPS signal quality metrics."""
        if not self.enable_signal_analysis:
            return 0.0, 0.0, 0.0, 0.0

        # Compute power spectrum
        fft_result = np.fft.fft(samples)
        power_spectrum = np.abs(fft_result) ** 2
        power_db = 10 * np.log10(power_spectrum + 1e-10)  # Add small value to avoid log(0)

        # Signal power (average)
        signal_power = np.mean(power_spectrum)
        signal_power_db = 10 * np.log10(signal_power + 1e-10)

        # Noise floor (estimate from lower percentiles)
        noise_floor = np.percentile(power_spectrum, 10)
        noise_floor_db = 10 * np.log10(noise_floor + 1e-10)

        # SNR
        snr_db = signal_power_db - noise_floor_db

        # Carrier power (power at center frequency)
        center_bin = len(power_spectrum) // 2
        carrier_power = power_spectrum[center_bin]
        carrier_power_db = 10 * np.log10(carrier_power + 1e-10)

        return signal_power_db, noise_floor_db, snr_db, carrier_power_db

    def publish_gps_data(self, samples):
        """Publish GPS signal data."""
        msg = GpsData()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'rtlsdr_gps_{self.device_index}'

        # Device info
        msg.device_index = self.device_index
        msg.serial_number = self.serial_number

        # GPS signal parameters
        msg.center_frequency = float(self.center_frequency)
        msg.sample_rate = float(self.sample_rate)
        msg.gain = float(self.sdr.gain) if self.gain_setting != 'auto' else 0.0
        msg.bandwidth = int(self.sample_rate)

        # GPS signal data
        msg.i_samples = samples.real.astype(np.float32).tolist()
        msg.q_samples = samples.imag.astype(np.float32).tolist()
        msg.num_samples = len(samples)

        # Signal quality analysis
        if self.enable_signal_analysis:
            signal_power_db, noise_floor_db, snr_db, carrier_power_db = \
                self.analyze_signal_quality(samples)
            msg.signal_power_db = float(signal_power_db)
            msg.noise_floor_db = float(noise_floor_db)
            msg.snr_db = float(snr_db)
            msg.carrier_power_db = float(carrier_power_db)
        else:
            msg.signal_power_db = 0.0
            msg.noise_floor_db = 0.0
            msg.snr_db = 0.0
            msg.carrier_power_db = 0.0

        # Timestamp and counter
        msg.timestamp_sec = time.time()
        msg.sample_counter = self.sample_counter
        self.sample_counter += len(samples)

        self.gps_publisher.publish(msg)

        # Log signal quality periodically (every 10 messages)
        if self.sample_counter % (self.num_samples * 10) < self.num_samples:
            self.get_logger().info(
                f'GPS Signal Quality - '
                f'Power: {msg.signal_power_db:.2f} dB, '
                f'SNR: {msg.snr_db:.2f} dB, '
                f'Carrier: {msg.carrier_power_db:.2f} dB'
            )

    def process_position_decoding(self, samples):
        """Process samples for GPS position decoding."""
        if not self.enable_position_decoding or self.gps_decoder is None:
            return

        try:
            # Buffer samples for better processing
            self.samples_buffer.append(samples)
            if len(self.samples_buffer) > self.buffer_size:
                self.samples_buffer.pop(0)

            # Use combined buffer for processing
            if len(self.samples_buffer) > 0:
                combined_samples = np.concatenate(self.samples_buffer)
            else:
                combined_samples = samples

            # Decode GPS position
            position_data = self.gps_decoder.process_samples(combined_samples)

            if position_data:
                # Publish position fix
                self.publish_position_fix(position_data)

        except Exception as e:
            self.get_logger().debug(f'GPS decoding error: {str(e)}')

    def publish_position_fix(self, position_data: dict):
        """Publish GPS position as NavSatFix message."""
        msg = NavSatFix()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        # Status
        fix_type = position_data.get('fix_type', 0)
        if fix_type >= 3:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif fix_type == 2:
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        msg.status.service = NavSatStatus.SERVICE_GPS

        # Position
        msg.latitude = position_data.get('latitude', 0.0)
        msg.longitude = position_data.get('longitude', 0.0)
        msg.altitude = position_data.get('altitude', 0.0)

        # Position covariance
        covariance = position_data.get('position_covariance', None)
        if covariance and len(covariance) == 9:
            msg.position_covariance = covariance
        else:
            # Default covariance based on HDOP
            hdop = position_data.get('hdop', 5.0)
            variance = (hdop * 5.0) ** 2  # Rough conversion
            vdop = position_data.get('vdop', hdop * 1.5)
            v_variance = (vdop * 5.0) ** 2
            msg.position_covariance = [
                variance, 0.0, 0.0,
                0.0, variance, 0.0,
                0.0, 0.0, v_variance
            ]

        msg.position_covariance_type = position_data.get(
            'position_covariance_type',
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        self.position_publisher.publish(msg)

        # Log position
        self.get_logger().info(
            f'GPS Position Fix: '
            f'Lat: {msg.latitude:.6f}°, '
            f'Lon: {msg.longitude:.6f}°, '
            f'Alt: {msg.altitude:.1f}m, '
            f'HDOP: {position_data.get("hdop", 0.0):.2f}, '
            f'Satellites: {position_data.get("satellites_used", 0)}, '
            f'Fix Type: {fix_type}'
        )

    def shutdown(self):
        """Shutdown the node and close the RTL-SDR device."""
        self.is_running = False
        if self.sdr is not None:
            try:
                self.sdr.close()
                self.get_logger().info('RTL-SDR device closed')
            except Exception as e:
                self.get_logger().error(f'Error closing RTL-SDR device: {str(e)}')

    def __del__(self):
        """Destructor."""
        self.shutdown()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = None

    try:
        node = GpsReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if node is not None:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception as e:
                print(f'Error during node cleanup: {str(e)}')
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Already shut down


if __name__ == '__main__':
    main()

