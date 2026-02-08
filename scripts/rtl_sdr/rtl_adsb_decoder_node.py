#!/usr/bin/env python3
"""
ADS-B Decoder Node for RTL-SDR V4

Decodes ADS-B (Automatic Dependent Surveillance-Broadcast) messages from
aircraft using the RTL-SDR Blog V4 at 1090 MHz.

Supports two decoding backends:
  1. dump1090 Mode (recommended): Launches dump1090 as a subprocess,
     parses SBS/BaseStation output on TCP port 30003.
  2. IQ Mode (fallback): Subscribes to IQ samples from rtl_sdr_node,
     demodulates and decodes using pyModeS.

Uses shared components from sdr_adsb_common for aircraft tracking,
IQ demodulation, and pyModeS decoding.

Topics Published:
  ~/aircraft      (sensor_msgs/NavSatFix): Individual aircraft position
  ~/aircraft_list (std_msgs/String): Summary of all tracked aircraft
  ~/messages      (std_msgs/String): Raw decoded messages (optional)

Parameters:
  mode (string): 'dump1090' or 'iq' [dump1090]
  dump1090_path (string): Path to dump1090 binary [dump1090]
  iq_topic (string): IQ samples topic for iq mode
  device_index (int): RTL-SDR device index [0]
  gain (float): Tuner gain in dB, -1 for auto [-1]
  enable_interactive (bool): Enable dump1090 interactive display [false]
  publish_raw_messages (bool): Publish raw decoded messages [false]
  max_aircraft (int): Maximum tracked aircraft [200]
  aircraft_timeout (float): Stale aircraft timeout in seconds [60.0]
  debug (bool): Enable debug logging [false]

Dependencies:
  - dump1090 (for dump1090 mode): https://github.com/antirez/dump1090
  - pyModeS (for iq mode): pip install pyModeS
"""

import os
import sys
import subprocess
import threading
import time
import numpy as np
from collections import deque
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import NavSatFix

# Add script install directory to path for shared module import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sdr_adsb_common import (
    Aircraft, AircraftTracker, ADSBIQDemodulator, PyModeSDecoder,
    PYMODES_AVAILABLE, publish_aircraft_navsatfix, publish_aircraft_list_string,
)


class RTLADSBDecoderNode(Node):
    """ROS2 node for ADS-B decoding with RTL-SDR V4."""

    def __init__(self):
        super().__init__('rtl_adsb_decoder_node')

        # ====================================================================
        # Parameters
        # ====================================================================

        self.declare_parameter('mode', 'dump1090')
        self.declare_parameter('dump1090_path', 'dump1090')
        self.declare_parameter('iq_topic', '/rtl_sdr/rtl_sdr_node/iq_samples')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('gain', -1.0)
        self.declare_parameter('enable_interactive', False)
        self.declare_parameter('publish_raw_messages', False)
        self.declare_parameter('max_aircraft', 200)
        self.declare_parameter('aircraft_timeout', 60.0)
        self.declare_parameter('debug', False)

        self.mode = self.get_parameter('mode').value
        self.dump1090_path = self.get_parameter('dump1090_path').value
        self.iq_topic = self.get_parameter('iq_topic').value
        self.device_index = self.get_parameter('device_index').value
        self.gain = self.get_parameter('gain').value
        self.enable_interactive = self.get_parameter('enable_interactive').value
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
        self.decoder = PyModeSDecoder(
            logger=self.get_logger(),
        )
        # IQ demodulator only needed in iq mode
        self.demodulator = None
        self.iq_buffer = deque(maxlen=int(2.4e6 * 0.1))

        # dump1090 state
        self.dump1090_process = None
        self.decoder_thread = None
        self.stop_flag = threading.Event()

        # ====================================================================
        # Publishers
        # ====================================================================

        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(
            String, '~/aircraft_list', 10)
        if self.publish_raw:
            self.raw_msg_pub = self.create_publisher(String, '~/messages', 10)

        # ====================================================================
        # Start decoder backend
        # ====================================================================

        if self.mode == 'dump1090':
            self._start_dump1090()
        elif self.mode == 'iq':
            if not PYMODES_AVAILABLE:
                self.get_logger().error(
                    'pyModeS required for IQ mode. '
                    'Install: pip install pyModeS')
                self.get_logger().error('Falling back to dump1090 mode.')
                self.mode = 'dump1090'
                self._start_dump1090()
            else:
                self.demodulator = ADSBIQDemodulator(
                    sample_rate=2.4e6,
                    logger=self.get_logger() if self.debug else None,
                )
                self.iq_sub = self.create_subscription(
                    Float32MultiArray, self.iq_topic,
                    self.iq_callback, 10)
                self.get_logger().info(
                    f'IQ mode: subscribing to {self.iq_topic}')
        else:
            self.get_logger().error(
                f'Unknown mode: {self.mode}. Use "dump1090" or "iq".')

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

        self.get_logger().info('RTL-SDR ADS-B Decoder Node initialized')
        self.get_logger().info(f'  Mode: {self.mode}')
        self.get_logger().info(f'  Device Index: {self.device_index}')
        self.get_logger().info(
            f'  Gain: {"auto" if self.gain < 0 else f"{self.gain:.1f} dB"}')

    # ====================================================================
    # dump1090 Mode
    # ====================================================================

    def _start_dump1090(self):
        """Start dump1090 as a subprocess and parse SBS output."""
        dump_bin = self.dump1090_path
        if not self._find_executable(dump_bin):
            alternatives = [
                'dump1090', 'dump1090-mutability', 'dump1090-fa', 'readsb']
            found = None
            for alt in alternatives:
                if self._find_executable(alt):
                    found = alt
                    break
            if found:
                dump_bin = found
                self.get_logger().info(f'Using {dump_bin} for ADS-B decoding')
            else:
                self.get_logger().error(
                    'dump1090 not found! Install with: '
                    'scripts/rtl_sdr/install_rtl_sdr.sh')
                return

        supported_flags = self._probe_dump1090_flags(dump_bin)

        cmd = [dump_bin]
        cmd.extend(['--device-index', str(self.device_index)])

        if self.gain >= 0:
            cmd.extend(['--gain', f'{self.gain:.1f}'])
        else:
            if 'enable-agc' in supported_flags:
                cmd.append('--enable-agc')

        cmd.append('--net')
        cmd.extend(['--net-sbs-port', '30003'])

        if not self.enable_interactive:
            if 'quiet' in supported_flags:
                cmd.append('--quiet')

        if 'aggressive' in supported_flags:
            cmd.append('--aggressive')

        self.get_logger().info(f'Starting: {" ".join(cmd)}')

        try:
            self.dump1090_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True)

            time.sleep(2.0)

            if self.dump1090_process.poll() is not None:
                stderr = self.dump1090_process.stderr.read()
                self.get_logger().error(
                    f'dump1090 failed to start: {stderr}')
                return

            self.decoder_thread = threading.Thread(
                target=self._sbs_reader_worker, daemon=True)
            self.decoder_thread.start()

            self.get_logger().info(
                'dump1090 started, parsing SBS output...')

        except Exception as e:
            self.get_logger().error(f'Failed to start dump1090: {e}')

    def _find_executable(self, name: str) -> bool:
        try:
            result = subprocess.run(
                ['which', name], capture_output=True, timeout=2.0)
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _probe_dump1090_flags(self, dump_bin: str) -> set:
        """Probe which CLI flags a dump1090 build supports."""
        supported = set()
        try:
            result = subprocess.run(
                [dump_bin, '--help'],
                capture_output=True, text=True, timeout=3.0)
            help_text = result.stdout + result.stderr
            for flag in ['quiet', 'enable-agc', 'aggressive',
                         'net-only', 'metric', 'no-fix']:
                if f'--{flag}' in help_text:
                    supported.add(flag)
            self.get_logger().info(
                f'dump1090 supported flags: {sorted(supported)}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not probe dump1090 flags: {e}')
        return supported

    # ====================================================================
    # SBS Parsing (dump1090 mode)
    # ====================================================================

    def _sbs_reader_worker(self):
        """Connect to dump1090 SBS port 30003 and parse messages."""
        import socket

        time.sleep(2.0)

        while not self.stop_flag.is_set():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect(('127.0.0.1', 30003))
                self.get_logger().info(
                    'Connected to dump1090 SBS port 30003')

                buffer = ''
                while not self.stop_flag.is_set():
                    try:
                        data = sock.recv(4096)
                        if not data:
                            break
                        buffer += data.decode('utf-8', errors='ignore')

                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self._parse_sbs_message(line)

                    except socket.timeout:
                        continue

            except (ConnectionRefusedError, OSError) as e:
                if not self.stop_flag.is_set():
                    self.get_logger().debug(
                        f'SBS connection failed: {e}, retrying in 3s...')
                    time.sleep(3.0)
            finally:
                try:
                    sock.close()
                except Exception:
                    pass

    def _parse_sbs_message(self, line: str):
        """Parse SBS/BaseStation format and update tracker.

        SBS format:
          MSG,<type>,,,<icao>,,,,,<callsign>,<alt>,<speed>,<heading>,
          <lat>,<lon>,<vrate>,<squawk>,...
        """
        try:
            parts = line.split(',')
            if len(parts) < 11 or parts[0] != 'MSG':
                return

            msg_type = parts[1].strip()
            icao = parts[4].strip()

            if not icao or len(icao) != 6:
                return

            with self.tracker.lock:
                ac, is_new = self.tracker.get_or_create(icao)

                # Type 1: Identification
                if msg_type == '1':
                    cs = parts[10].strip() if len(parts) > 10 else None
                    if cs:
                        if ac.callsign != cs:
                            self.get_logger().info(
                                f'[ID] {icao}: {cs}')
                        ac.callsign = cs

                # Type 3: Airborne position
                elif msg_type == '3':
                    alt = self._safe_float(
                        parts[11]) if len(parts) > 11 else None
                    lat = self._safe_float(
                        parts[14]) if len(parts) > 14 else None
                    lon = self._safe_float(
                        parts[15]) if len(parts) > 15 else None

                    if alt is not None:
                        ac.altitude = alt
                    if lat is not None and lon is not None:
                        ac.latitude = lat
                        ac.longitude = lon
                        self.get_logger().info(
                            f'[POS] {icao}: ({lat:.6f}, {lon:.6f}) '
                            f'alt={alt:.0f}ft')
                        publish_aircraft_navsatfix(
                            ac, self.get_clock(), self.aircraft_pub)

                # Type 4: Airborne velocity
                elif msg_type == '4':
                    spd = self._safe_float(
                        parts[12]) if len(parts) > 12 else None
                    hdg = self._safe_float(
                        parts[13]) if len(parts) > 13 else None
                    vrate = self._safe_float(
                        parts[16]) if len(parts) > 16 else None

                    if spd is not None:
                        ac.speed = spd
                    if hdg is not None:
                        ac.heading = hdg
                    if vrate is not None:
                        ac.vertical_rate = vrate

                # Type 5: Surface position
                elif msg_type == '5':
                    lat = self._safe_float(
                        parts[14]) if len(parts) > 14 else None
                    lon = self._safe_float(
                        parts[15]) if len(parts) > 15 else None
                    if lat is not None and lon is not None:
                        ac.latitude = lat
                        ac.longitude = lon
                        ac.altitude = 0.0
                        ac.on_ground = True
                        publish_aircraft_navsatfix(
                            ac, self.get_clock(), self.aircraft_pub)

                # Type 6: Squawk
                elif msg_type == '6':
                    sq = parts[17].strip() if len(parts) > 17 else None
                    if sq:
                        ac.squawk = sq

                # Publish raw
                if self.publish_raw:
                    raw = String()
                    raw.data = line
                    self.raw_msg_pub.publish(raw)

        except Exception as e:
            if self.debug:
                self.get_logger().debug(
                    f'SBS parse error: {e} -- line: {line}')

    @staticmethod
    def _safe_float(s: str) -> Optional[float]:
        try:
            s = s.strip()
            return float(s) if s else None
        except (ValueError, TypeError):
            return None

    # ====================================================================
    # IQ Mode (fallback)
    # ====================================================================

    def iq_callback(self, msg):
        """Buffer IQ samples and process for ADS-B (IQ mode)."""
        try:
            iq_data = np.array(msg.data, dtype=np.float32)
            if len(iq_data) < 2:
                return

            i_samples = iq_data[0::2]
            q_samples = iq_data[1::2]
            iq = i_samples + 1j * q_samples

            self.iq_buffer.extend(iq)

            if len(self.iq_buffer) > int(2.4e6 * 0.05):
                self._process_iq_buffer()

        except Exception as e:
            self.get_logger().error(f'IQ callback error: {e}')

    def _process_iq_buffer(self):
        """Demodulate IQ buffer and decode ADS-B messages."""
        try:
            iq_array = np.array(list(self.iq_buffer))

            hex_messages = self.demodulator.demodulate(iq_array)

            for hex_msg in hex_messages:
                if self.debug:
                    self.get_logger().info(f'[DEMOD] {hex_msg}')

                ac = self.decoder.decode(hex_msg, self.tracker)

                if ac is not None:
                    publish_aircraft_navsatfix(
                        ac, self.get_clock(), self.aircraft_pub)

                    if self.publish_raw:
                        raw = String()
                        raw.data = f'{ac.icao_address}: {hex_msg}'
                        self.raw_msg_pub.publish(raw)

            # Trim buffer
            sample_rate = 2.4e6
            if len(self.iq_buffer) > int(sample_rate * 0.1):
                keep = int(sample_rate * 0.02)
                buf = list(self.iq_buffer)
                self.iq_buffer.clear()
                self.iq_buffer.extend(buf[-keep:])

        except Exception as e:
            self.get_logger().error(f'IQ processing error: {e}')

    # ====================================================================
    # Timer Callbacks
    # ====================================================================

    def _cleanup_callback(self):
        self.tracker.cleanup_stale()
        self.decoder.prune_cpr_buffer()

    def _publish_list_callback(self):
        publish_aircraft_list_string(self.tracker, self.aircraft_list_pub)

    # ====================================================================
    # Cleanup
    # ====================================================================

    def destroy_node(self):
        self.stop_flag.set()
        if self.dump1090_process and self.dump1090_process.poll() is None:
            self.dump1090_process.terminate()
            try:
                self.dump1090_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.dump1090_process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RTLADSBDecoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
