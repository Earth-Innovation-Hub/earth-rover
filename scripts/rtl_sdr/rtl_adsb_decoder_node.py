#!/usr/bin/env python3
"""
ADS-B Decoder Node for RTL-SDR V4

Decodes ADS-B (Automatic Dependent Surveillance-Broadcast) messages from
aircraft using the RTL-SDR Blog V4 at 1090 MHz.

This node can operate in two modes:
  1. IQ Mode:  Subscribe to IQ samples from rtl_sdr_node and decode in-process
  2. dump1090 Mode:  Launch dump1090 as a subprocess for high-performance
     decoding and parse its output (recommended for reliable ADS-B)

dump1090 is the de-facto standard for RTL-SDR ADS-B decoding and delivers
significantly better detection rates than raw IQ demodulation.

Topics Published:
  ~/aircraft      (sensor_msgs/NavSatFix): Individual aircraft position updates
  ~/aircraft_list (std_msgs/String): Summary of all tracked aircraft
  ~/messages      (std_msgs/String): Raw decoded messages (optional, debug)

Parameters:
  mode (string): 'dump1090' or 'iq' [dump1090]
  dump1090_path (string): Path to dump1090 binary [dump1090]
  iq_topic (string): IQ samples topic for iq mode
  device_index (int): RTL-SDR device index [0]
  gain (float): Tuner gain in dB, -1 for auto [-1]
  enable_interactive (bool): Enable dump1090 interactive stdout [false]
  publish_raw_messages (bool): Publish raw decoded messages [false]
  max_aircraft (int): Maximum tracked aircraft [200]
  aircraft_timeout (float): Stale aircraft timeout in seconds [60.0]

Dependencies:
  - dump1090 (recommended): https://github.com/antirez/dump1090
    or dump1090-mutability / dump1090-fa / readsb
  - pyModeS (for iq mode fallback): pip install pyModeS
"""

import os
import subprocess
import threading
import time
import re
import json
import numpy as np
from collections import defaultdict, deque
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import NavSatFix

# Optional: pyModeS for IQ-mode fallback decoding
try:
    import pyModeS as pms
    PYMODES_AVAILABLE = True
except ImportError:
    PYMODES_AVAILABLE = False

# Optional: scipy for signal filtering
try:
    from scipy import signal as scipy_signal
    SCIPY_AVAILABLE = True
except (ImportError, AttributeError):
    SCIPY_AVAILABLE = False


class Aircraft:
    """Represents a tracked aircraft."""

    def __init__(self, icao_address: str):
        self.icao_address = icao_address
        self.callsign = None
        self.latitude = None
        self.longitude = None
        self.altitude = None       # feet
        self.speed = None           # knots
        self.heading = None         # degrees
        self.vertical_rate = None   # ft/min
        self.squawk = None
        self.last_update = time.time()
        self.message_count = 0

    def update(self):
        self.last_update = time.time()
        self.message_count += 1

    def is_stale(self, timeout: float) -> bool:
        return (time.time() - self.last_update) > timeout

    def has_position(self) -> bool:
        return self.latitude is not None and self.longitude is not None


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
        self.max_aircraft = self.get_parameter('max_aircraft').value
        self.timeout = self.get_parameter('aircraft_timeout').value
        self.debug = self.get_parameter('debug').value

        # ====================================================================
        # State
        # ====================================================================

        self.aircraft: Dict[str, Aircraft] = {}
        self.lock = threading.Lock()
        self.dump1090_process = None
        self.decoder_thread = None
        self.stop_flag = threading.Event()

        # For IQ-mode preamble detection
        self.iq_buffer = deque(maxlen=int(2.4e6 * 0.1))  # 100ms at 2.4 MSPS
        self.position_buffer = {}

        # ====================================================================
        # Publishers
        # ====================================================================

        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(
            String, '~/aircraft_list', 10)
        if self.publish_raw:
            self.raw_msg_pub = self.create_publisher(String, '~/messages', 10)

        # ====================================================================
        # Start decoder
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
                self.iq_sub = self.create_subscription(
                    Float32MultiArray,
                    self.iq_topic,
                    self.iq_callback,
                    10
                )
                self.get_logger().info(
                    f'IQ mode: subscribing to {self.iq_topic}')
        else:
            self.get_logger().error(
                f'Unknown mode: {self.mode}. Use "dump1090" or "iq".')

        # ====================================================================
        # Timers
        # ====================================================================

        self.cleanup_timer = self.create_timer(
            10.0, self.cleanup_stale_aircraft)
        self.publish_timer = self.create_timer(
            1.0, self.publish_aircraft_list)

        self.get_logger().info('RTL-SDR ADS-B Decoder Node initialized')
        self.get_logger().info(f'  Mode: {self.mode}')
        self.get_logger().info(f'  Device Index: {self.device_index}')
        self.get_logger().info(
            f'  Gain: {"auto" if self.gain < 0 else f"{self.gain:.1f} dB"}')

    # ====================================================================
    # dump1090 Mode
    # ====================================================================

    def _start_dump1090(self):
        """Start dump1090 as a subprocess and parse its SBS/BaseStation output."""
        # Check if dump1090 is available
        dump_bin = self.dump1090_path
        if not self._find_executable(dump_bin):
            # Try common alternative names
            alternatives = [
                'dump1090', 'dump1090-mutability', 'dump1090-fa', 'readsb'
            ]
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
                self.get_logger().error(
                    'Or manually: '
                    'https://github.com/antirez/dump1090')
                return

        # Probe which flags this dump1090 build supports.
        # The original antirez dump1090, dump1090-mutability, dump1090-fa,
        # and readsb all have slightly different CLIs.
        supported_flags = self._probe_dump1090_flags(dump_bin)

        # Build command
        cmd = [dump_bin]

        # Device index
        cmd.extend(['--device-index', str(self.device_index)])

        # Gain: original antirez dump1090 uses --gain -100 for auto,
        # mutability/fa/readsb use --enable-agc or --gain -10
        if self.gain >= 0:
            cmd.extend(['--gain', f'{self.gain:.1f}'])
        else:
            if 'enable-agc' in supported_flags:
                cmd.append('--enable-agc')
            else:
                # Fallback: max gain (omit --gain entirely, dump1090
                # defaults to max gain which is the best default for ADS-B)
                pass

        # Enable networking (SBS BaseStation output on port 30003)
        cmd.append('--net')
        cmd.extend(['--net-sbs-port', '30003'])

        # Suppress interactive display if the flag is supported
        if not self.enable_interactive:
            if 'quiet' in supported_flags:
                cmd.append('--quiet')
            # If --quiet isn't supported (antirez original), dump1090 will
            # just print to stderr which we ignore anyway

        # Aggressive mode for better decoding if available
        if 'aggressive' in supported_flags:
            cmd.append('--aggressive')

        self.get_logger().info(f'Starting: {" ".join(cmd)}')

        try:
            self.dump1090_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            time.sleep(2.0)

            if self.dump1090_process.poll() is not None:
                stderr = self.dump1090_process.stderr.read()
                self.get_logger().error(f'dump1090 failed to start: {stderr}')
                return

            # Start SBS parsing thread (connects to port 30003)
            self.decoder_thread = threading.Thread(
                target=self._sbs_reader_worker, daemon=True)
            self.decoder_thread.start()

            self.get_logger().info('dump1090 started, parsing SBS output...')

        except Exception as e:
            self.get_logger().error(f'Failed to start dump1090: {e}')

    def _find_executable(self, name: str) -> bool:
        """Check if an executable exists on PATH."""
        try:
            result = subprocess.run(
                ['which', name], capture_output=True, timeout=2.0)
            return result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _probe_dump1090_flags(self, dump_bin: str) -> set:
        """Probe which CLI flags a dump1090 build supports.

        Different forks have different flags:
          antirez/dump1090:       --aggressive, --net, --gain, NO --quiet
          dump1090-mutability:    --quiet, --enable-agc, --net, --gain
          dump1090-fa / readsb:   --quiet, --enable-agc, --net, --gain
        """
        supported = set()
        try:
            result = subprocess.run(
                [dump_bin, '--help'],
                capture_output=True, text=True, timeout=3.0
            )
            help_text = result.stdout + result.stderr

            # Check for specific flags in --help output
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

    def _sbs_reader_worker(self):
        """Connect to dump1090 SBS port 30003 and parse messages."""
        import socket

        # Give dump1090 a moment to open its network port
        time.sleep(2.0)

        while not self.stop_flag.is_set():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect(('127.0.0.1', 30003))
                self.get_logger().info('Connected to dump1090 SBS port 30003')

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
        """Parse a SBS/BaseStation format message.

        SBS format (comma-separated):
          MSG,<type>,,,<icao>,,,,,<callsign>,<alt>,<speed>,<heading>,
          <lat>,<lon>,<vrate>,<squawk>,<alert>,<emergency>,<spi>,<ground>
        """
        try:
            parts = line.split(',')

            if len(parts) < 11 or parts[0] != 'MSG':
                return

            msg_type = parts[1].strip()
            icao = parts[4].strip()

            if not icao or len(icao) != 6:
                return

            with self.lock:
                if icao not in self.aircraft:
                    if len(self.aircraft) >= self.max_aircraft:
                        oldest = min(
                            self.aircraft.items(),
                            key=lambda x: x[1].last_update)
                        del self.aircraft[oldest[0]]
                    self.aircraft[icao] = Aircraft(icao)
                    self.get_logger().info(f'[NEW] Aircraft ICAO: {icao}')

                ac = self.aircraft[icao]
                ac.update()

                # Parse based on message type
                # Type 1: Identification (callsign)
                if msg_type == '1':
                    cs = parts[10].strip() if len(parts) > 10 else None
                    if cs:
                        if ac.callsign != cs:
                            self.get_logger().info(
                                f'[UPDATE] {icao}: Callsign = {cs}')
                        ac.callsign = cs

                # Type 3: Airborne position
                elif msg_type == '3':
                    alt = self._safe_float(parts[11]) if len(parts) > 11 else None
                    lat = self._safe_float(parts[14]) if len(parts) > 14 else None
                    lon = self._safe_float(parts[15]) if len(parts) > 15 else None

                    if alt is not None:
                        ac.altitude = alt
                    if lat is not None and lon is not None:
                        ac.latitude = lat
                        ac.longitude = lon
                        self.get_logger().info(
                            f'[UPDATE] {icao}: Position = '
                            f'({lat:.6f}, {lon:.6f}) Alt = {alt:.0f}ft')
                        self.publish_aircraft(ac)

                # Type 4: Airborne velocity
                elif msg_type == '4':
                    spd = self._safe_float(parts[12]) if len(parts) > 12 else None
                    hdg = self._safe_float(parts[13]) if len(parts) > 13 else None
                    vrate = self._safe_float(parts[16]) if len(parts) > 16 else None

                    if spd is not None:
                        ac.speed = spd
                    if hdg is not None:
                        ac.heading = hdg
                    if vrate is not None:
                        ac.vertical_rate = vrate

                # Type 5: Surface position
                elif msg_type == '5':
                    lat = self._safe_float(parts[14]) if len(parts) > 14 else None
                    lon = self._safe_float(parts[15]) if len(parts) > 15 else None
                    if lat is not None and lon is not None:
                        ac.latitude = lat
                        ac.longitude = lon
                        ac.altitude = 0.0
                        self.publish_aircraft(ac)

                # Type 6: Squawk
                elif msg_type == '6':
                    sq = parts[17].strip() if len(parts) > 17 else None
                    if sq:
                        ac.squawk = sq

                # Publish raw message if enabled
                if self.publish_raw:
                    msg = String()
                    msg.data = line
                    self.raw_msg_pub.publish(msg)

        except Exception as e:
            if self.debug:
                self.get_logger().debug(f'SBS parse error: {e} -- line: {line}')

    @staticmethod
    def _safe_float(s: str) -> Optional[float]:
        """Safely convert string to float, returning None on failure."""
        try:
            s = s.strip()
            if s == '':
                return None
            return float(s)
        except (ValueError, TypeError):
            return None

    # ====================================================================
    # IQ Mode (fallback -- uses pyModeS)
    # ====================================================================

    def iq_callback(self, msg):
        """Process incoming IQ samples for ADS-B decoding (IQ mode)."""
        if not PYMODES_AVAILABLE:
            return

        try:
            iq_data = np.array(msg.data, dtype=np.float32)
            if len(iq_data) < 2:
                return

            i_samples = iq_data[0::2]
            q_samples = iq_data[1::2]
            iq = i_samples + 1j * q_samples

            self.iq_buffer.extend(iq)

            if len(self.iq_buffer) > int(2.4e6 * 0.05):
                self._process_iq_adsb()

        except Exception as e:
            self.get_logger().error(f'IQ callback error: {e}')

    def _process_iq_adsb(self):
        """Process IQ buffer for ADS-B (IQ mode with pyModeS)."""
        try:
            iq_array = np.array(list(self.iq_buffer))
            power = np.abs(iq_array)

            if SCIPY_AVAILABLE and len(power) > 100:
                try:
                    b, a = scipy_signal.butter(3, 0.1, 'low')
                    power = scipy_signal.filtfilt(b, a, power)
                except Exception:
                    pass

            # Simplified preamble search and decoding
            # (same approach as hydra_sdr adsb_decoder_node)
            sample_rate = 2.4e6
            samples_per_bit = sample_rate / 1e6

            normalized = power / (np.max(power) + 1e-12)
            threshold = np.mean(normalized) + 0.3 * (
                np.max(normalized) - np.mean(normalized))

            search_len = min(len(normalized), int(20 * samples_per_bit))

            for start in range(search_len):
                # Quick energy check for preamble region
                preamble_len = int(16 * samples_per_bit)
                if start + preamble_len + 112 * int(samples_per_bit) > len(normalized):
                    break

                preamble_region = normalized[start:start + preamble_len]
                if np.mean(preamble_region) < threshold * 0.5:
                    continue

                # Try to extract 112 bits
                msg_start = start + preamble_len
                bits = []
                bit_dur = int(samples_per_bit)

                for i in range(112):
                    bs = msg_start + i * bit_dur
                    be = bs + bit_dur
                    if be > len(power):
                        break
                    seg = normalized[bs:be]
                    mid = len(seg) // 2
                    first = np.mean(seg[:mid])
                    second = np.mean(seg[mid:])
                    if first > second:
                        bits.append(0)
                    else:
                        bits.append(1)

                if len(bits) == 112:
                    hex_chars = []
                    for i in range(0, 112, 4):
                        nibble = bits[i:i+4]
                        val = int(''.join(map(str, nibble)), 2)
                        hex_chars.append(f'{val:X}')
                    hex_msg = ''.join(hex_chars)

                    if len(hex_msg) == 28:
                        self._decode_pymodes(hex_msg)
                    break

            # Trim buffer
            if len(self.iq_buffer) > int(sample_rate * 0.1):
                keep = int(sample_rate * 0.02)
                buf = list(self.iq_buffer)
                self.iq_buffer.clear()
                self.iq_buffer.extend(buf[-keep:])

        except Exception as e:
            self.get_logger().error(f'IQ ADS-B processing error: {e}')

    def _decode_pymodes(self, hex_msg: str):
        """Decode an ADS-B hex message using pyModeS."""
        try:
            df = pms.df(hex_msg)
            if df != 17:
                return

            icao = pms.icao(hex_msg)
            tc = pms.typecode(hex_msg)

            with self.lock:
                if icao not in self.aircraft:
                    if len(self.aircraft) >= self.max_aircraft:
                        oldest = min(
                            self.aircraft.items(),
                            key=lambda x: x[1].last_update)
                        del self.aircraft[oldest[0]]
                    self.aircraft[icao] = Aircraft(icao)
                    self.get_logger().info(f'[NEW] Aircraft ICAO: {icao}')

                ac = self.aircraft[icao]
                ac.update()

                if 1 <= tc <= 4:
                    cs = pms.callsign(hex_msg)
                    if cs:
                        ac.callsign = cs
                        self.get_logger().info(
                            f'[UPDATE] {icao}: Callsign = {cs}')

                elif 9 <= tc <= 18:
                    alt = pms.altitude(hex_msg)
                    if alt:
                        ac.altitude = alt

                elif 19 <= tc <= 22:
                    vel = pms.velocity(hex_msg)
                    if vel:
                        ac.speed, ac.heading, ac.vertical_rate = vel[0], vel[1], vel[2]

                self.publish_aircraft(ac)

                if self.publish_raw:
                    msg = String()
                    msg.data = f'{icao}: {hex_msg}'
                    self.raw_msg_pub.publish(msg)

        except Exception as e:
            if self.debug:
                self.get_logger().debug(f'pyModeS decode error: {e}')

    # ====================================================================
    # Publishing
    # ====================================================================

    def publish_aircraft(self, ac: Aircraft):
        """Publish aircraft position as NavSatFix."""
        if not ac.has_position():
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'aircraft_{ac.icao_address}'
        msg.latitude = ac.latitude
        msg.longitude = ac.longitude
        msg.altitude = ac.altitude if ac.altitude else 0.0
        msg.status.status = 1
        msg.status.service = 1

        self.aircraft_pub.publish(msg)

    def publish_aircraft_list(self):
        """Publish summary of all tracked aircraft."""
        with self.lock:
            entries = []
            for icao, ac in self.aircraft.items():
                info = icao
                if ac.callsign:
                    info += f' ({ac.callsign})'
                if ac.altitude is not None:
                    info += f' @{ac.altitude:.0f}ft'
                if ac.speed is not None:
                    info += f' {ac.speed:.0f}kts'
                entries.append(info)

            msg = String()
            msg.data = (
                f'Tracked Aircraft ({len(entries)}): '
                + ', '.join(entries)
            )
            self.aircraft_list_pub.publish(msg)

    def cleanup_stale_aircraft(self):
        """Remove aircraft not seen recently."""
        with self.lock:
            stale = [
                icao for icao, ac in self.aircraft.items()
                if ac.is_stale(self.timeout)
            ]
            for icao in stale:
                del self.aircraft[icao]
                self.get_logger().info(f'Removed stale aircraft: {icao}')

    # ====================================================================
    # Cleanup
    # ====================================================================

    def destroy_node(self):
        """Clean shutdown."""
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
