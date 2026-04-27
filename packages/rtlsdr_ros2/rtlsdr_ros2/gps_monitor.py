#!/usr/bin/env python3
"""
GPS Monitor Node for RTL-SDR ROS2 Package
Monitors and displays GPS signal quality metrics in real-time.
"""

import rclpy
from rclpy.node import Node
from rtlsdr_ros2.msg import GpsData
import numpy as np


class GpsMonitorNode(Node):
    """ROS2 Node for monitoring GPS signal quality from RTL-SDR."""

    def __init__(self):
        super().__init__('gps_monitor')

        # Statistics tracking
        self.msg_count = 0
        self.snr_history = []
        self.power_history = []
        self.max_history_size = 100

        # Create subscriber
        self.gps_subscriber = self.create_subscription(
            GpsData,
            'rtlsdr/gps_data',
            self.gps_callback,
            10
        )

        self.get_logger().info('GPS Monitor Node started')
        self.get_logger().info('Subscribing to: /rtlsdr/gps_data')
        self.get_logger().info('Monitoring GPS L1 signal quality...')

    def gps_callback(self, msg):
        """Process received GPS data."""
        self.msg_count += 1

        # Update history
        self.snr_history.append(msg.snr_db)
        self.power_history.append(msg.signal_power_db)
        if len(self.snr_history) > self.max_history_size:
            self.snr_history.pop(0)
            self.power_history.pop(0)

        # Log detailed info every 10th message
        if self.msg_count % 10 == 0:
            avg_snr = np.mean(self.snr_history) if self.snr_history else 0.0
            avg_power = np.mean(self.power_history) if self.power_history else 0.0
            max_snr = max(self.snr_history) if self.snr_history else 0.0
            min_snr = min(self.snr_history) if self.snr_history else 0.0

            self.get_logger().info(
                f'\n=== GPS Signal Quality Report ===\n'
                f'Device: {msg.device_index} ({msg.serial_number})\n'
                f'Frequency: {msg.center_frequency/1e6:.6f} MHz (GPS L1)\n'
                f'Sample Rate: {msg.sample_rate/1e6:.3f} MSps\n'
                f'Gain: {msg.gain:.1f} dB\n'
                f'\n--- Current Signal Metrics ---\n'
                f'Signal Power: {msg.signal_power_db:.2f} dB\n'
                f'Noise Floor: {msg.noise_floor_db:.2f} dB\n'
                f'SNR: {msg.snr_db:.2f} dB\n'
                f'Carrier Power: {msg.carrier_power_db:.2f} dB\n'
                f'\n--- Statistics (last {len(self.snr_history)} samples) ---\n'
                f'Average SNR: {avg_snr:.2f} dB\n'
                f'Peak SNR: {max_snr:.2f} dB\n'
                f'Min SNR: {min_snr:.2f} dB\n'
                f'Average Power: {avg_power:.2f} dB\n'
                f'\n--- Sample Info ---\n'
                f'Samples: {msg.num_samples}\n'
                f'Total Messages: {self.msg_count}\n'
                f'Sample Counter: {msg.sample_counter}'
            )

            # Signal quality assessment
            if msg.snr_db > 10.0:
                self.get_logger().info('✓ Good GPS signal quality detected!')
            elif msg.snr_db > 5.0:
                self.get_logger().warn('⚠ Moderate GPS signal quality')
            else:
                self.get_logger().warn('✗ Weak GPS signal - check antenna and positioning')

            # GPS acquisition tips
            if msg.snr_db < 3.0:
                self.get_logger().warn(
                    'GPS Acquisition Tips:\n'
                    '  - Ensure antenna has clear view of sky\n'
                    '  - Use GPS patch antenna if available\n'
                    '  - Try increasing gain (20-40 dB)\n'
                    '  - Move to outdoor location\n'
                    '  - Wait 30-60 seconds for GPS lock'
                )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        node = GpsMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

