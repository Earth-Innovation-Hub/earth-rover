#!/usr/bin/env python3
"""
Spectrum Visualizer Node for RTL-SDR ROS2 Package
This node subscribes to spectrum data and provides logging/monitoring capabilities.
"""

import rclpy
from rclpy.node import Node
from rtlsdr_ros2.msg import RtlsdrSpectrum
import numpy as np


class SpectrumVisualizerNode(Node):
    """ROS2 Node for visualizing and monitoring RTL-SDR spectrum data."""

    def __init__(self):
        super().__init__('spectrum_visualizer')

        # Create subscriber
        self.spectrum_subscriber = self.create_subscription(
            RtlsdrSpectrum,
            'rtlsdr/spectrum',
            self.spectrum_callback,
            10
        )

        self.get_logger().info('Spectrum Visualizer Node started')
        self.get_logger().info('Subscribing to: /rtlsdr/spectrum')

        self.msg_count = 0

    def spectrum_callback(self, msg):
        """Process received spectrum data."""
        self.msg_count += 1

        # Log every 10th message to avoid spam
        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f'\n=== RTL-SDR Spectrum Data ===\n'
                f'Device: {msg.device_index} ({msg.serial_number})\n'
                f'Center Frequency: {msg.center_frequency/1e6:.3f} MHz\n'
                f'Sample Rate: {msg.sample_rate/1e6:.3f} MSps\n'
                f'Bandwidth: {msg.bandwidth/1e6:.3f} MHz\n'
                f'FFT Size: {msg.fft_size}\n'
                f'Peak Frequency: {msg.peak_frequency/1e6:.6f} MHz\n'
                f'Peak Power: {msg.peak_power_db:.2f} dB\n'
                f'Average Power: {msg.average_power_db:.2f} dB\n'
                f'Messages Received: {self.msg_count}'
            )

            # Find top 5 peaks
            power_db = np.array(msg.power_db)
            frequencies = np.array(msg.frequencies)
            
            # Get indices of top 5 peaks
            top_indices = np.argsort(power_db)[-5:][::-1]
            
            self.get_logger().info('Top 5 Frequencies:')
            for i, idx in enumerate(top_indices, 1):
                self.get_logger().info(
                    f'  {i}. {frequencies[idx]/1e6:.6f} MHz: {power_db[idx]:.2f} dB'
                )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        node = SpectrumVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

