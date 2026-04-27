#!/usr/bin/env python3
"""
Signal Recorder Node for RTL-SDR ROS2 Package
Records IQ samples to file for offline analysis.
"""

import rclpy
from rclpy.node import Node
from rtlsdr_ros2.msg import RtlsdrSignal
import numpy as np
import json
from datetime import datetime
import os


class SignalRecorderNode(Node):
    """ROS2 Node for recording RTL-SDR signal data to file."""

    def __init__(self):
        super().__init__('signal_recorder')

        # Declare parameters
        self.declare_parameter('output_directory', '/tmp/rtlsdr_recordings')
        self.declare_parameter('max_samples', 1000000)  # Max samples to record
        self.declare_parameter('auto_start', True)
        
        # Get parameters
        self.output_directory = self.get_parameter('output_directory').value
        self.max_samples = self.get_parameter('max_samples').value
        self.auto_start = self.get_parameter('auto_start').value

        # Create output directory
        os.makedirs(self.output_directory, exist_ok=True)

        # Recording state
        self.is_recording = self.auto_start
        self.recorded_samples = []
        self.total_samples = 0
        self.metadata = None
        
        # Generate filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_filename = f'rtlsdr_recording_{timestamp}'

        # Create subscriber
        self.signal_subscriber = self.create_subscription(
            RtlsdrSignal,
            'rtlsdr/signal',
            self.signal_callback,
            10
        )

        self.get_logger().info('Signal Recorder Node started')
        self.get_logger().info(f'Output directory: {self.output_directory}')
        self.get_logger().info(f'Recording: {"ON" if self.is_recording else "OFF"}')
        self.get_logger().info(f'Max samples: {self.max_samples}')

    def signal_callback(self, msg):
        """Process received signal data."""
        if not self.is_recording:
            return

        # Store metadata from first message
        if self.metadata is None:
            self.metadata = {
                'device_index': msg.device_index,
                'serial_number': msg.serial_number,
                'center_frequency': msg.center_frequency,
                'sample_rate': msg.sample_rate,
                'gain': msg.gain,
                'bandwidth': msg.bandwidth,
                'timestamp_start': msg.timestamp_sec,
            }
            self.get_logger().info(
                f'Started recording at {msg.center_frequency/1e6:.3f} MHz'
            )

        # Convert to complex samples
        i_samples = np.array(msg.i_samples)
        q_samples = np.array(msg.q_samples)
        complex_samples = i_samples + 1j * q_samples
        
        self.recorded_samples.append(complex_samples)
        self.total_samples += len(complex_samples)

        # Log progress every 100k samples
        if self.total_samples % 100000 < msg.num_samples:
            self.get_logger().info(
                f'Recorded {self.total_samples}/{self.max_samples} samples'
            )

        # Stop if max samples reached
        if self.total_samples >= self.max_samples:
            self.get_logger().info('Max samples reached, stopping recording')
            self.stop_recording()

    def stop_recording(self):
        """Stop recording and save data to file."""
        if not self.is_recording:
            return

        self.is_recording = False
        
        if self.total_samples == 0:
            self.get_logger().warn('No samples recorded')
            return

        # Concatenate all samples
        all_samples = np.concatenate(self.recorded_samples)
        
        # Save IQ data
        iq_filename = os.path.join(
            self.output_directory,
            f'{self.base_filename}.npy'
        )
        np.save(iq_filename, all_samples)
        self.get_logger().info(f'Saved IQ data to: {iq_filename}')

        # Save metadata
        if self.metadata:
            self.metadata['total_samples'] = self.total_samples
            self.metadata['duration_sec'] = (
                self.total_samples / self.metadata['sample_rate']
            )
            
            metadata_filename = os.path.join(
                self.output_directory,
                f'{self.base_filename}_metadata.json'
            )
            with open(metadata_filename, 'w') as f:
                json.dump(self.metadata, f, indent=2)
            self.get_logger().info(f'Saved metadata to: {metadata_filename}')

        # Also save as complex64 binary for compatibility
        bin_filename = os.path.join(
            self.output_directory,
            f'{self.base_filename}.complex64'
        )
        all_samples.astype(np.complex64).tofile(bin_filename)
        self.get_logger().info(f'Saved binary data to: {bin_filename}')

        self.get_logger().info(
            f'\nRecording Summary:\n'
            f'  Total samples: {self.total_samples}\n'
            f'  Duration: {self.metadata["duration_sec"]:.2f} seconds\n'
            f'  Center freq: {self.metadata["center_frequency"]/1e6:.3f} MHz\n'
            f'  Sample rate: {self.metadata["sample_rate"]/1e6:.3f} MSps'
        )

    def __del__(self):
        """Destructor - ensure data is saved."""
        if self.is_recording and self.total_samples > 0:
            self.get_logger().info('Node shutting down, saving recorded data...')
            self.stop_recording()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        node = SignalRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            if node.is_recording and node.total_samples > 0:
                node.get_logger().info('Saving recorded data...')
                node.stop_recording()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

