#!/usr/bin/env python3
"""
RTL-SDR Reader Node for ROS2
This node interfaces with RTL-SDR devices and publishes IQ samples and spectrum data.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
from rtlsdr import RtlSdr
import threading
import time

from rtlsdr_ros2.msg import RtlsdrSignal, RtlsdrSpectrum
from std_msgs.msg import Header


class RtlsdrReaderNode(Node):
    """ROS2 Node for reading RTL-SDR device data."""

    def __init__(self):
        super().__init__('rtlsdr_reader')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_index', 0, ParameterDescriptor(description='RTL-SDR device index')),
                ('center_frequency', 100.0e6, ParameterDescriptor(description='Center frequency in Hz')),
                ('sample_rate', 2.048e6, ParameterDescriptor(description='Sample rate in Hz')),
                ('gain', 'auto', ParameterDescriptor(description='Gain setting (auto or value in dB)')),
                ('num_samples', 1024, ParameterDescriptor(description='Number of samples per read')),
                ('publish_rate', 10.0, ParameterDescriptor(description='Publishing rate in Hz')),
                ('publish_raw_iq', True, ParameterDescriptor(description='Publish raw IQ samples')),
                ('publish_spectrum', True, ParameterDescriptor(description='Publish spectrum data')),
                ('fft_size', 1024, ParameterDescriptor(description='FFT size for spectrum analysis')),
            ]
        )

        # Get parameters
        self.device_index = self.get_parameter('device_index').value
        self.center_frequency = self.get_parameter('center_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.gain_setting = self.get_parameter('gain').value
        self.num_samples = self.get_parameter('num_samples').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_raw_iq = self.get_parameter('publish_raw_iq').value
        self.publish_spectrum = self.get_parameter('publish_spectrum').value
        self.fft_size = self.get_parameter('fft_size').value

        # Create publishers
        if self.publish_raw_iq:
            self.signal_publisher = self.create_publisher(
                RtlsdrSignal,
                'rtlsdr/signal',
                10
            )
            self.get_logger().info('Publishing raw IQ samples on topic: rtlsdr/signal')

        if self.publish_spectrum:
            self.spectrum_publisher = self.create_publisher(
                RtlsdrSpectrum,
                'rtlsdr/spectrum',
                10
            )
            self.get_logger().info('Publishing spectrum data on topic: rtlsdr/spectrum')

        # Initialize RTL-SDR device
        self.sdr = None
        self.serial_number = "unknown"
        self.is_running = False
        self.init_device()

        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('RTL-SDR Reader Node initialized')

    def init_device(self):
        """Initialize the RTL-SDR device."""
        try:
            self.sdr = RtlSdr(self.device_index)

            # Configure device
            self.sdr.sample_rate = self.sample_rate
            self.sdr.center_freq = self.center_frequency

            # Set gain
            if self.gain_setting == 'auto':
                self.sdr.gain = 'auto'
                self.get_logger().info('Gain set to auto')
            else:
                self.sdr.gain = float(self.gain_setting)
                self.get_logger().info(f'Gain set to {self.gain_setting} dB')

            # Get device info
            try:
                # Note: serial number might not be available on all devices
                self.serial_number = str(self.device_index)
            except Exception:
                self.serial_number = f"device_{self.device_index}"

            self.is_running = True

            self.get_logger().info(
                f'RTL-SDR device initialized:\n'
                f'  Device Index: {self.device_index}\n'
                f'  Center Frequency: {self.center_frequency/1e6:.3f} MHz\n'
                f'  Sample Rate: {self.sample_rate/1e6:.3f} MSps\n'
                f'  Gain: {self.gain_setting}\n'
                f'  Serial Number: {self.serial_number}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to initialize RTL-SDR device: {str(e)}')
            self.get_logger().error('Please ensure RTL-SDR device is connected and drivers are installed.')
            self.is_running = False

    def timer_callback(self):
        """Timer callback to read and publish RTL-SDR data."""
        if not self.is_running or self.sdr is None:
            return

        try:
            # Read IQ samples from RTL-SDR
            samples = self.sdr.read_samples(self.num_samples)

            # Publish raw IQ samples if enabled
            if self.publish_raw_iq:
                self.publish_iq_samples(samples)

            # Publish spectrum data if enabled
            if self.publish_spectrum:
                self.publish_spectrum_data(samples)

        except Exception as e:
            self.get_logger().error(f'Error reading from RTL-SDR: {str(e)}')

    def publish_iq_samples(self, samples):
        """Publish raw IQ samples."""
        msg = RtlsdrSignal()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'rtlsdr_{self.device_index}'

        # Device info
        msg.device_index = self.device_index
        msg.serial_number = self.serial_number

        # Tuner settings
        msg.center_frequency = float(self.center_frequency)
        msg.sample_rate = float(self.sample_rate)
        msg.gain = float(self.sdr.gain) if self.gain_setting != 'auto' else 0.0
        msg.bandwidth = int(self.sample_rate)

        # Signal data
        msg.i_samples = samples.real.astype(np.float32).tolist()
        msg.q_samples = samples.imag.astype(np.float32).tolist()
        msg.num_samples = len(samples)

        # Timestamp
        msg.timestamp_sec = time.time()

        self.signal_publisher.publish(msg)

    def publish_spectrum_data(self, samples):
        """Compute and publish spectrum data from IQ samples."""
        # Ensure we have enough samples for FFT
        if len(samples) < self.fft_size:
            return

        # Take only fft_size samples
        fft_samples = samples[:self.fft_size]

        # Apply window function (Hann window)
        window = np.hanning(self.fft_size)
        windowed_samples = fft_samples * window

        # Compute FFT
        fft_result = np.fft.fft(windowed_samples)
        fft_shifted = np.fft.fftshift(fft_result)

        # Compute magnitude and power
        magnitude = np.abs(fft_shifted)
        power_db = 20 * np.log10(magnitude + 1e-10)  # Add small value to avoid log(0)

        # Compute frequency bins
        freq_bins = np.fft.fftshift(np.fft.fftfreq(self.fft_size, 1.0/self.sample_rate))
        frequencies = self.center_frequency + freq_bins

        # Find peak
        peak_idx = np.argmax(power_db)
        peak_frequency = frequencies[peak_idx]
        peak_power = power_db[peak_idx]
        average_power = np.mean(power_db)

        # Create and publish message
        msg = RtlsdrSpectrum()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'rtlsdr_{self.device_index}'

        # Device info
        msg.device_index = self.device_index
        msg.serial_number = self.serial_number

        # Tuner settings
        msg.center_frequency = float(self.center_frequency)
        msg.sample_rate = float(self.sample_rate)
        msg.bandwidth = float(self.sample_rate)

        # Spectrum data
        msg.frequencies = frequencies.astype(np.float32).tolist()
        msg.magnitudes = magnitude.astype(np.float32).tolist()
        msg.power_db = power_db.astype(np.float32).tolist()
        msg.fft_size = self.fft_size

        # Statistics
        msg.peak_frequency = float(peak_frequency)
        msg.peak_power_db = float(peak_power)
        msg.average_power_db = float(average_power)

        self.spectrum_publisher.publish(msg)

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

    try:
        node = RtlsdrReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
