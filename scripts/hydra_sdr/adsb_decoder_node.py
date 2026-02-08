#!/usr/bin/env python3
"""
ADS-B Decoder Node for HydraSDR

Decodes ADS-B (Automatic Dependent Surveillance-Broadcast) messages from
aircraft using the HydraSDR RFOne at 1090 MHz via IQ sample demodulation.

Uses shared components from sdr_adsb_common for aircraft tracking,
IQ demodulation, and pyModeS decoding.

Topics Subscribed:
  (via iq_topic param) IQ samples from SDR node

Topics Published:
  ~/aircraft      (sensor_msgs/NavSatFix): Individual aircraft position
  ~/aircraft_list (std_msgs/String): Summary of all tracked aircraft
  ~/messages      (std_msgs/String): Raw decoded messages (optional)

Parameters:
  iq_topic (string): IQ samples topic [/hydra_sdr/hydra_sdr_node/iq_samples]
  sample_rate (float): Sample rate in Hz [2.5e6]
  center_frequency (float): Center frequency in Hz [1090e6]
  enable_decoding (bool): Enable message decoding [true]
  publish_raw_messages (bool): Publish raw hex messages [false]
  max_aircraft (int): Maximum tracked aircraft [200]
  aircraft_timeout (float): Stale aircraft timeout in seconds [60.0]
  debug (bool): Enable debug logging [false]

Dependencies:
  - pyModeS: pip install pyModeS
  - numpy
  - scipy (optional, improves signal filtering)
"""

import os
import sys
import time
import numpy as np
from collections import deque
from typing import Optional
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import NavSatFix

# Add script install directory to path for shared module import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdr_adsb_common import (
    Aircraft, AircraftTracker, ADSBIQDemodulator, PyModeSDecoder,
    PYMODES_AVAILABLE, publish_aircraft_navsatfix, publish_aircraft_list_string,
)


class ADSBDecoderNode(Node):
    """ROS2 node for decoding ADS-B from HydraSDR IQ samples."""

    def __init__(self):
        super().__init__('adsb_decoder_node')

        # ====================================================================
        # Parameters
        # ====================================================================

        self.declare_parameter('iq_topic', '/hydra_sdr/hydra_sdr_node/iq_samples')
        self.declare_parameter('sample_rate', 2.5e6)
        self.declare_parameter('center_frequency', 1090e6)
        self.declare_parameter('enable_decoding', True)
        self.declare_parameter('publish_raw_messages', False)
        self.declare_parameter('max_aircraft', 200)
        self.declare_parameter('aircraft_timeout', 60.0)
        self.declare_parameter('debug', False)

        iq_topic = self.get_parameter('iq_topic').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.center_freq = self.get_parameter('center_frequency').value
        self.enable_decoding = self.get_parameter('enable_decoding').value
        self.publish_raw = self.get_parameter('publish_raw_messages').value
        max_aircraft = self.get_parameter('max_aircraft').value
        timeout = self.get_parameter('aircraft_timeout').value
        self.debug = self.get_parameter('debug').value

        # ====================================================================
        # Shared components
        # ====================================================================

        self.tracker = AircraftTracker(
            max_aircraft=max_aircraft,
            timeout=timeout,
            logger=self.get_logger(),
        )
        self.demodulator = ADSBIQDemodulator(
            sample_rate=self.sample_rate,
            logger=self.get_logger() if self.debug else None,
        )
        self.decoder = PyModeSDecoder(
            logger=self.get_logger(),
        )

        # IQ sample buffer (100ms worth)
        self.iq_buffer = deque(maxlen=int(self.sample_rate * 0.1))

        # ====================================================================
        # Publishers
        # ====================================================================

        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(
            String, '~/aircraft_list', 10)
        if self.publish_raw:
            self.raw_msg_pub = self.create_publisher(String, '~/messages', 10)

        # ====================================================================
        # Subscriber
        # ====================================================================

        if PYMODES_AVAILABLE and self.enable_decoding:
            self.iq_sub = self.create_subscription(
                Float32MultiArray, iq_topic, self.iq_callback, 10)
        else:
            self.get_logger().warn(
                'ADS-B decoding disabled -- pyModeS not available')

        # ====================================================================
        # Timers
        # ====================================================================

        self.cleanup_timer = self.create_timer(
            10.0, self._cleanup_callback)
        self.publish_timer = self.create_timer(
            1.0, self._publish_list_callback)

        # ====================================================================
        # Startup log
        # ====================================================================

        self.get_logger().info('ADS-B Decoder Node initialized (HydraSDR)')
        self.get_logger().info(f'  IQ Topic: {iq_topic}')
        self.get_logger().info(
            f'  Sample Rate: {self.sample_rate / 1e6:.1f} MSPS')
        self.get_logger().info(
            f'  Center Frequency: {self.center_freq / 1e6:.3f} MHz')
        self.get_logger().info(
            f'  Decoding: '
            f'{"Enabled" if PYMODES_AVAILABLE and self.enable_decoding else "Disabled"}')

        if abs(self.center_freq - 1090e6) > 1e6:
            self.get_logger().warn(
                f'  Center frequency is {self.center_freq / 1e6:.3f} MHz, '
                f'expected 1090 MHz for ADS-B!')

    # ====================================================================
    # IQ Callback
    # ====================================================================

    def iq_callback(self, msg):
        """Buffer incoming IQ samples and process when enough collected."""
        try:
            iq_data = np.array(msg.data, dtype=np.float32)
            if len(iq_data) < 2:
                return

            i_samples = iq_data[0::2]
            q_samples = iq_data[1::2]
            iq = i_samples + 1j * q_samples

            self.iq_buffer.extend(iq)

            # Process when we have at least 50ms of data
            if len(self.iq_buffer) > int(self.sample_rate * 0.05):
                self._process_buffer()

        except Exception as e:
            self.get_logger().error(f'IQ callback error: {e}')

    def _process_buffer(self):
        """Demodulate IQ buffer and decode any ADS-B messages found."""
        try:
            iq_array = np.array(list(self.iq_buffer))

            # Demodulate: extract hex messages from IQ
            hex_messages = self.demodulator.demodulate(iq_array)

            for hex_msg in hex_messages:
                if self.debug:
                    self.get_logger().info(f'[DEMOD] {hex_msg}')

                # Decode via pyModeS and update tracker
                ac = self.decoder.decode(hex_msg, self.tracker)

                if ac is not None:
                    # Publish position if available
                    publish_aircraft_navsatfix(
                        ac, self.get_clock(), self.aircraft_pub)

                    # Publish raw message if enabled
                    if self.publish_raw:
                        raw = String()
                        raw.data = f'{ac.icao_address}: {hex_msg}'
                        self.raw_msg_pub.publish(raw)

            # Trim buffer: keep last 20ms for overlap
            if len(self.iq_buffer) > int(self.sample_rate * 0.1):
                keep = int(self.sample_rate * 0.02)
                buf = list(self.iq_buffer)
                self.iq_buffer.clear()
                self.iq_buffer.extend(buf[-keep:])

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # ====================================================================
    # Timer Callbacks
    # ====================================================================

    def _cleanup_callback(self):
        self.tracker.cleanup_stale()
        self.decoder.prune_cpr_buffer()

    def _publish_list_callback(self):
        publish_aircraft_list_string(self.tracker, self.aircraft_list_pub)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ADSBDecoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
