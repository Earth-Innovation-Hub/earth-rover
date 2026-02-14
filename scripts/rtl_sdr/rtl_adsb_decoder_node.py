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
import math
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
    estimate_receiver_position,
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
        # Receiver-position Kalman filter state (ENU local frame)
        self._kf_initialized = False
        self._kf_ref_lat = None
        self._kf_ref_lon = None
        self._kf_x = np.zeros((4, 1), dtype=np.float64)   # [x, y, vx, vy]
        self._kf_P = np.eye(4, dtype=np.float64) * 1e6
        self._kf_last_t = None

        # ====================================================================
        # Publishers
        # ====================================================================

        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(
            String, '~/aircraft_list', 10)
        self.estimated_pos_pub = self.create_publisher(
            String, '~/estimated_position', 10)
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

        # Cleanup stale aircraft every 5 seconds (aircraft_timeout param
        # controls how long they stay; default 60s = 1 minute of no data).
        self.cleanup_timer = self.create_timer(
            5.0, self._cleanup_callback)
        # Publish aircraft list + re-broadcast all positions every 2 seconds.
        self.publish_timer = self.create_timer(
            2.0, self._publish_list_callback)
        # Estimate receiver position from aircraft constellation every 5s.
        self.estimate_timer = self.create_timer(
            5.0, self._estimate_position_callback)

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
            # IMPORTANT: Do NOT use subprocess.PIPE for stdout/stderr.
            # dump1090 writes status output continuously.  If the pipe
            # buffers fill up (64 KB) and nobody reads them, dump1090's
            # write() blocks and it stops processing radio data entirely.
            # We read ADS-B data from TCP :30003, not stdout, so discard
            # dump1090's console output.
            self._dump1090_log = open('/tmp/dump1090_stderr.log', 'w')
            self.dump1090_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=self._dump1090_log)

            time.sleep(2.0)

            if self.dump1090_process.poll() is not None:
                # dump1090 exited immediately -- read the log for clues
                self._dump1090_log.flush()
                try:
                    with open('/tmp/dump1090_stderr.log', 'r') as f:
                        err = f.read(4096)
                    self.get_logger().error(
                        f'dump1090 failed to start: {err}')
                except Exception:
                    self.get_logger().error(
                        'dump1090 failed to start (no stderr captured)')
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
        """Periodically publish aircraft list AND re-publish all positions.

        This ensures the VCS (or any late subscriber) always receives
        up-to-date positions for every tracked aircraft, not just when
        a new SBS position message arrives from dump1090.
        """
        publish_aircraft_list_string(self.tracker, self.aircraft_list_pub)

        # Re-publish NavSatFix for every aircraft that has a position.
        # The tracker lock is acquired inside aircraft_list_string() above
        # and released, so acquire again here for the position sweep.
        with self.tracker.lock:
            for ac in self.tracker.aircraft.values():
                if ac.has_position():
                    publish_aircraft_navsatfix(
                        ac, self.get_clock(), self.aircraft_pub)

    def _estimate_position_callback(self):
        """Estimate receiver position from the aircraft constellation."""
        import json

        result = estimate_receiver_position(self.tracker, min_aircraft=4)
        if result is None:
            return

        # Apply receiver Kalman filtering on top of robust estimator output.
        raw_lat = float(result.get('lat'))
        raw_lon = float(result.get('lon'))
        meas_sigma_km = result.get('sigma_major_km')
        if meas_sigma_km is None:
            # Fallback if only radius is available.
            meas_sigma_km = max(1.0, float(result.get('est_radius_km', 20.0)) * 0.5)
        kf_lat, kf_lon, kf_radius_km = self._kf_update_receiver_position(
            raw_lat, raw_lon, float(meas_sigma_km) * 1000.0)

        result['raw_lat'] = round(raw_lat, 6)
        result['raw_lon'] = round(raw_lon, 6)
        result['kf_lat'] = round(kf_lat, 6)
        result['kf_lon'] = round(kf_lon, 6)
        result['kf_radius_km'] = round(kf_radius_km, 2)
        # Keep compatibility for existing consumers (map reads lat/lon).
        result['lat'] = result['kf_lat']
        result['lon'] = result['kf_lon']
        result['est_radius_km'] = result['kf_radius_km']

        # Publish as JSON string for easy parsing on the VCS side
        msg = String()
        msg.data = json.dumps(result)
        self.estimated_pos_pub.publish(msg)

        est_lat = result.get('lat', 0.0)
        est_lon = result.get('lon', 0.0)
        count = result.get('aircraft_used', 0)
        confidence = result.get('confidence', 0.0)
        radius_km = result.get('est_radius_km', 0.0)
        self.get_logger().info(
            f'[EST] Receiver position: ({est_lat:.5f}, {est_lon:.5f}) '
            f'from {count} aircraft, confidence={confidence:.0%}, '
            f'radius~{radius_km:.1f}km')

    def _kf_latlon_to_xy(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to local ENU meters around filter reference."""
        if self._kf_ref_lat is None or self._kf_ref_lon is None:
            self._kf_ref_lat = lat
            self._kf_ref_lon = lon
            return 0.0, 0.0
        earth_r = 6371000.0
        lat0_rad = math.radians(self._kf_ref_lat)
        x = earth_r * math.cos(lat0_rad) * math.radians(lon - self._kf_ref_lon)
        y = earth_r * math.radians(lat - self._kf_ref_lat)
        return x, y

    def _kf_xy_to_latlon(self, x: float, y: float) -> tuple:
        """Convert local ENU meters back to lat/lon."""
        earth_r = 6371000.0
        lat0 = self._kf_ref_lat if self._kf_ref_lat is not None else 0.0
        lon0 = self._kf_ref_lon if self._kf_ref_lon is not None else 0.0
        lat = lat0 + math.degrees(y / earth_r)
        lon = lon0 + math.degrees(
            x / (earth_r * max(1e-9, math.cos(math.radians(lat0)))))
        return lat, lon

    def _kf_update_receiver_position(
            self, meas_lat: float, meas_lon: float, meas_sigma_m: float) -> tuple:
        """Constant-velocity 2D Kalman update for receiver estimate.

        Returns:
            (kf_lat, kf_lon, kf_radius_km)
        """
        now_t = time.time()
        mx, my = self._kf_latlon_to_xy(meas_lat, meas_lon)

        if not self._kf_initialized:
            self._kf_x = np.array([[mx], [my], [0.0], [0.0]], dtype=np.float64)
            p_pos = max(100.0, meas_sigma_m) ** 2
            self._kf_P = np.diag([p_pos, p_pos, 2500.0, 2500.0]).astype(np.float64)
            self._kf_last_t = now_t
            self._kf_initialized = True
            lat, lon = self._kf_xy_to_latlon(mx, my)
            return lat, lon, max(1.0, 2.0 * meas_sigma_m / 1000.0)

        dt = max(0.1, min(30.0, now_t - (self._kf_last_t or now_t)))
        self._kf_last_t = now_t

        # State transition
        A = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        # Process noise: white acceleration model
        q_acc = 1.5  # m/s^2
        q = q_acc * q_acc
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = q * np.array([
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ], dtype=np.float64)

        # Predict
        self._kf_x = A @ self._kf_x
        self._kf_P = A @ self._kf_P @ A.T + Q

        # Measurement update (x,y only)
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=np.float64)
        r_var = max(100.0, meas_sigma_m) ** 2
        R = np.array([[r_var, 0.0], [0.0, r_var]], dtype=np.float64)
        z = np.array([[mx], [my]], dtype=np.float64)
        y = z - (H @ self._kf_x)
        S = H @ self._kf_P @ H.T + R
        K = self._kf_P @ H.T @ np.linalg.inv(S)
        self._kf_x = self._kf_x + (K @ y)
        I = np.eye(4, dtype=np.float64)
        self._kf_P = (I - K @ H) @ self._kf_P

        fx = float(self._kf_x[0, 0])
        fy = float(self._kf_x[1, 0])
        kf_lat, kf_lon = self._kf_xy_to_latlon(fx, fy)

        pos_cov = self._kf_P[:2, :2]
        evals = np.linalg.eigvalsh(pos_cov)
        sigma_major_m = math.sqrt(max(float(evals[-1]), 0.0))
        kf_radius_km = max(1.0, (2.0 * sigma_major_m) / 1000.0)
        return kf_lat, kf_lon, kf_radius_km

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
        if hasattr(self, '_dump1090_log') and self._dump1090_log:
            try:
                self._dump1090_log.close()
            except Exception:
                pass
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
