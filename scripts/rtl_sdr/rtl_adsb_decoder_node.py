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
  ~/aircraft       (sensor_msgs/NavSatFix): Individual aircraft position
  ~/aircraft_list  (std_msgs/String): JSON list of all tracked aircraft with flight + radio details
  ~/radio_summary  (std_msgs/String): JSON global radio summary and histograms
  ~/estimated_position (std_msgs/String): JSON receiver position estimate (KF)
  ~/messages       (std_msgs/String): Raw decoded messages (optional)

Parameters:
  mode (string): 'dump1090' or 'iq' [dump1090]
  dump1090_path (string): Path to dump1090 binary [dump1090]
  iq_topic (string): IQ samples topic for iq mode
  device_index (int): RTL-SDR device index [0]
  gain (float): Tuner gain in dB, -1 for auto [-1]
  enable_interactive (bool): Enable dump1090 interactive display [false]
  dump1090_http_port (int): dump1090 web map HTTP port [8082] (8080 = default dump1090)
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
    PYMODES_AVAILABLE, publish_aircraft_navsatfix,
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
        self.declare_parameter('dump1090_http_port', 8082)  # dump1090 web map port (8080 often used by others)
        self.declare_parameter('publish_raw_messages', False)
        self.declare_parameter('max_aircraft', 200)
        self.declare_parameter('aircraft_timeout', 60.0)
        self.declare_parameter('debug', False)
        self.declare_parameter('reference_gps_topic', '/mavros/global_position/raw/fix')
        self.declare_parameter('kf_process_accel_mps2', 1.5)
        self.declare_parameter('kf_innovation_gate', 9.21)  # chi2(2 dof, 99%)
        self.declare_parameter('estimator_active_window_s', 15.0)
        self.declare_parameter('estimator_time_decay_s', 30.0)

        self.mode = self.get_parameter('mode').value
        self.dump1090_path = self.get_parameter('dump1090_path').value
        self.iq_topic = self.get_parameter('iq_topic').value
        self.device_index = self.get_parameter('device_index').value
        self.gain = self.get_parameter('gain').value
        self.enable_interactive = self.get_parameter('enable_interactive').value
        self.dump1090_http_port = int(self.get_parameter('dump1090_http_port').value)
        self.publish_raw = self.get_parameter('publish_raw_messages').value
        max_aircraft = self.get_parameter('max_aircraft').value
        timeout = self.get_parameter('aircraft_timeout').value
        self.debug = self.get_parameter('debug').value
        self.reference_gps_topic = self.get_parameter('reference_gps_topic').value
        self.kf_process_accel_mps2 = float(
            self.get_parameter('kf_process_accel_mps2').value)
        self.kf_innovation_gate = float(
            self.get_parameter('kf_innovation_gate').value)
        self.estimator_active_window_s = float(
            self.get_parameter('estimator_active_window_s').value)
        self.estimator_time_decay_s = float(
            self.get_parameter('estimator_time_decay_s').value)

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
        self._kf_nis_window = deque(maxlen=120)
        self._kf_reject_window = deque(maxlen=120)
        self._reference_lat = None
        self._reference_lon = None
        self._raw_err_window_m = deque(maxlen=120)
        self._kf_err_window_m = deque(maxlen=120)
        # Aircraft-level radio/decoder telemetry (windowed counters)
        self._radio_lock = threading.Lock()
        self._radio_window_s = 60.0
        self._radio_stats = {}  # icao -> telemetry dict

        # ====================================================================
        # Publishers
        # ====================================================================

        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(
            String, '~/aircraft_list', 10)
        self.estimated_pos_pub = self.create_publisher(
            String, '~/estimated_position', 10)
        self.radio_summary_pub = self.create_publisher(
            String, '~/radio_summary', 10)
        self.reference_gps_sub = self.create_subscription(
            NavSatFix, self.reference_gps_topic, self._reference_gps_callback, 10)
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
        # Publish aircraft list (flight + radio) and radio summary every 2 s; re-broadcast positions.
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
        self.get_logger().info(f'  Reference GPS Topic: {self.reference_gps_topic}')
        self.get_logger().info(
            f'  Estimator Active Window: {self.estimator_active_window_s:.1f}s')
        self.get_logger().info(
            f'  Radio Metrics Window: {self._radio_window_s:.0f}s')

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

        # Use a different HTTP port so 8080 is free (e.g. for web_video_server or other tools)
        if self.dump1090_http_port != 8080:
            if 'net-http-port' in supported_flags:
                cmd.extend(['--net-http-port', str(self.dump1090_http_port)])
            elif 'web-server-port' in supported_flags:
                cmd.extend(['--web-server-port', str(self.dump1090_http_port)])
            else:
                self.get_logger().warn(
                    'dump1090 does not support --net-http-port or --web-server-port; '
                    'web map will use default port 8080')

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
                         'net-only', 'metric', 'no-fix',
                         'net-http-port', 'web-server-port']:
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

                # Update aircraft-level radio/decoder telemetry.
                self._update_radio_stats(icao, msg_type, ac)

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

    def _update_radio_stats(self, icao: str, msg_type: str, ac: Aircraft):
        """Update rolling per-aircraft message/radio telemetry."""
        now = time.time()
        with self._radio_lock:
            if icao not in self._radio_stats:
                self._radio_stats[icao] = {
                    'all_ts': deque(maxlen=1200),
                    'type_ts': {
                        '1': deque(maxlen=600),  # ID
                        '3': deque(maxlen=600),  # airborne pos
                        '4': deque(maxlen=600),  # velocity
                        '5': deque(maxlen=600),  # surface pos
                        '6': deque(maxlen=600),  # squawk
                    },
                    'last_seen': now,
                    'first_seen': now,
                    'callsign': None,
                }
            st = self._radio_stats[icao]
            st['all_ts'].append(now)
            if msg_type in st['type_ts']:
                st['type_ts'][msg_type].append(now)
            st['last_seen'] = now
            if ac.callsign:
                st['callsign'] = ac.callsign

    @staticmethod
    def _prune_ts_deque(ts_deque: deque, cutoff_t: float):
        while ts_deque and ts_deque[0] < cutoff_t:
            ts_deque.popleft()

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
        removed = self.tracker.cleanup_stale()
        if removed:
            with self._radio_lock:
                for icao in removed:
                    self._radio_stats.pop(icao, None)
        self.decoder.prune_cpr_buffer()

    def _publish_list_callback(self):
        """Publish combined aircraft list (flight + radio) as JSON, radio summary, and re-broadcast positions."""
        import json

        now = time.time()
        cutoff = now - self._radio_window_s

        with self._radio_lock:
            for st in self._radio_stats.values():
                self._prune_ts_deque(st['all_ts'], cutoff)
                for d in st['type_ts'].values():
                    self._prune_ts_deque(d, cutoff)
            stale_keys = [
                icao for icao, st in self._radio_stats.items()
                if len(st['all_ts']) == 0 and (now - st['last_seen']) > self._radio_window_s
            ]
            for icao in stale_keys:
                del self._radio_stats[icao]
            radio_snapshot = {}
            for icao, st in self._radio_stats.items():
                radio_snapshot[icao] = {
                    'msg_count': len(st['all_ts']),
                    'pos_count': len(st['type_ts']['3']) + len(st['type_ts']['5']),
                    'vel_count': len(st['type_ts']['4']),
                    'id_count': len(st['type_ts']['1']),
                    'last_seen': st['last_seen'],
                    'callsign': st['callsign'],
                }

        with self.tracker.lock:
            tracker_list = list(self.tracker.aircraft.items())

        aircraft_rows = []
        for icao, ac in tracker_list:
            rs = radio_snapshot.get(icao, {})
            age_s = max(0.0, now - rs.get('last_seen', now))
            msg_rate_hz = rs.get('msg_count', 0) / max(1.0, self._radio_window_s)
            pos_rate_hz = rs.get('pos_count', 0) / max(1.0, self._radio_window_s)
            vel_rate_hz = rs.get('vel_count', 0) / max(1.0, self._radio_window_s)
            id_rate_hz = rs.get('id_count', 0) / max(1.0, self._radio_window_s)
            has_position = ac.has_position()
            has_speed = ac.speed is not None
            has_heading = ac.heading is not None
            freshness = max(0.0, 1.0 - age_s / 20.0)
            rate_score = min(msg_rate_hz / 1.2, 1.0)
            completeness = (
                (0.4 if has_position else 0.0)
                + (0.2 if has_speed else 0.0)
                + (0.2 if has_heading else 0.0)
                + (0.2 if (ac.callsign or rs.get('callsign')) else 0.0)
            )
            radio_quality = max(0.0, min(1.0, 0.45 * freshness + 0.35 * rate_score + 0.20 * completeness))

            row = {
                'icao': icao,
                'callsign': ac.callsign or rs.get('callsign'),
                'latitude': ac.latitude,
                'longitude': ac.longitude,
                'altitude': ac.altitude,
                'speed': ac.speed,
                'heading': ac.heading,
                'vertical_rate': ac.vertical_rate,
                'last_update': ac.last_update,
                'has_position': has_position,
                'has_speed': has_speed,
                'has_heading': has_heading,
                'age_s': round(age_s, 1),
                'msg_count_60s': rs.get('msg_count', 0),
                'msg_rate_hz': round(msg_rate_hz, 3),
                'pos_rate_hz': round(pos_rate_hz, 3),
                'vel_rate_hz': round(vel_rate_hz, 3),
                'id_rate_hz': round(id_rate_hz, 3),
                'radio_quality': round(radio_quality, 3),
                'radio_quality_pct': int(round(radio_quality * 100.0)),
                'rssi_dbfs': None,
                'snr_db': None,
            }
            aircraft_rows.append(row)

        aircraft_rows.sort(key=lambda r: (r['msg_rate_hz'], r.get('last_update') or 0), reverse=True)
        aircraft_rows = aircraft_rows[:200]

        # Publish combined aircraft list (flight + radio)
        list_msg = String()
        list_msg.data = json.dumps({
            'tracked_count': len(tracker_list),
            'updated_at': round(now, 3),
            'window_s': self._radio_window_s,
            'mode': self.mode,
            'aircraft': aircraft_rows,
        })
        self.aircraft_list_pub.publish(list_msg)

        # Radio summary (histograms and global stats)
        rates = [r['msg_rate_hz'] for r in aircraft_rows]
        ages = [r['age_s'] for r in aircraft_rows]
        qualities = [r['radio_quality_pct'] for r in aircraft_rows]
        total_msgs_60s = sum(r['msg_count_60s'] for r in aircraft_rows)

        def hist_counts(values, edges):
            counts = [0] * (len(edges) - 1)
            for v in values:
                for i in range(len(edges) - 1):
                    if (i == len(edges) - 2 and edges[i] <= v <= edges[i + 1]) or (edges[i] <= v < edges[i + 1]):
                        counts[i] += 1
                        break
                else:
                    if counts:
                        counts[0 if v < edges[0] else -1] += 1
            return counts

        rate_edges = [0.0, 0.1, 0.3, 0.7, 1.5, 5.0]
        age_edges = [0.0, 2.0, 5.0, 10.0, 20.0, 60.0]
        quality_edges = [0, 20, 40, 60, 80, 100]
        summary = {
            'mode': self.mode,
            'window_s': self._radio_window_s,
            'updated_at': round(now, 3),
            'total_aircraft': len(aircraft_rows),
            'active_aircraft_lt10s': sum(1 for r in aircraft_rows if r['age_s'] < 10.0),
            'total_msgs_60s': int(total_msgs_60s),
            'global_msg_rate_hz': round(total_msgs_60s / self._radio_window_s, 3),
            'median_aircraft_rate_hz': round(float(np.median(rates)) if rates else 0.0, 3),
            'median_age_s': round(float(np.median(ages)) if ages else 0.0, 2),
            'mean_quality_pct': round(float(np.mean(qualities)) if qualities else 0.0, 1),
            'rate_hist': {'edges': rate_edges, 'counts': hist_counts(rates, rate_edges)},
            'age_hist': {'edges': age_edges, 'counts': hist_counts(ages, age_edges)},
            'quality_hist': {'edges': quality_edges, 'counts': hist_counts(qualities, quality_edges)},
            'fields_note': {'rssi_dbfs': 'Not available in dump1090 SBS mode', 'snr_db': 'Not available in dump1090 SBS mode'},
        }
        summary_msg = String()
        summary_msg.data = json.dumps(summary)
        self.radio_summary_pub.publish(summary_msg)

        # Re-publish NavSatFix for every aircraft that has a position
        with self.tracker.lock:
            for ac in self.tracker.aircraft.values():
                if ac.has_position():
                    publish_aircraft_navsatfix(
                        ac, self.get_clock(), self.aircraft_pub)

    def _reference_gps_callback(self, msg: NavSatFix):
        """Cache reference rover GPS position for estimator diagnostics."""
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if lat == 0.0 and lon == 0.0:
            return
        self._reference_lat = lat
        self._reference_lon = lon

    @staticmethod
    def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance (meters) between two lat/lon points."""
        r = 6371000.0
        p1 = math.radians(lat1)
        p2 = math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = (math.sin(dp / 2.0) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(1e-12, 1.0 - a)))
        return r * c

    def _radio_quality_for_icaos(self, icaos: list) -> float:
        """Compute mean radio quality proxy [0..1] for selected ICAOs."""
        if not icaos:
            return 0.5
        now = time.time()
        qualities = []
        with self._radio_lock:
            for icao in icaos:
                st = self._radio_stats.get(icao)
                if not st:
                    continue
                age_s = max(0.0, now - float(st.get('last_seen', now)))
                msg_count = len(st.get('all_ts', []))
                msg_rate_hz = msg_count / max(1.0, self._radio_window_s)
                freshness = max(0.0, 1.0 - age_s / 20.0)
                rate_score = min(msg_rate_hz / 1.2, 1.0)
                qualities.append(0.5 * freshness + 0.5 * rate_score)
        if not qualities:
            return 0.5
        return float(np.mean(qualities))

    def _estimate_position_callback(self):
        """Estimate receiver position from the aircraft constellation."""
        import json

        result = estimate_receiver_position(
            self.tracker,
            min_aircraft=4,
            time_decay_s=self.estimator_time_decay_s,
            active_window_s=self.estimator_active_window_s,
        )
        if result is None:
            return

        # Apply receiver Kalman filtering on top of robust estimator output.
        raw_lat = float(result.get('lat'))
        raw_lon = float(result.get('lon'))
        meas_sigma_km = result.get('sigma_major_km')
        if meas_sigma_km is None:
            # Fallback if only radius is available.
            meas_sigma_km = max(1.0, float(result.get('est_radius_km', 20.0)) * 0.5)

        # Leverage aircraft-level radio characteristics:
        # high-quality aircraft constellation => trust measurement more.
        used_icaos = result.get('used_icaos', [])
        radio_quality_mean = self._radio_quality_for_icaos(used_icaos)
        sigma_scale = max(0.70, min(1.30, 1.30 - 0.60 * radio_quality_mean))
        meas_sigma_km *= sigma_scale

        kf_lat, kf_lon, kf_radius_km, kf_nis, kf_accepted = self._kf_update_receiver_position(
            raw_lat, raw_lon, float(meas_sigma_km) * 1000.0)

        result['raw_lat'] = round(raw_lat, 6)
        result['raw_lon'] = round(raw_lon, 6)
        result['kf_lat'] = round(kf_lat, 6)
        result['kf_lon'] = round(kf_lon, 6)
        result['kf_radius_km'] = round(kf_radius_km, 2)
        result['kf_nis'] = round(float(kf_nis), 3)
        result['kf_update_accepted'] = bool(kf_accepted)
        result['kf_gate_threshold'] = self.kf_innovation_gate
        result['radio_quality_mean'] = round(radio_quality_mean, 3)
        result['kf_sigma_scale'] = round(sigma_scale, 3)

        # Rolling KF health metrics.
        self._kf_nis_window.append(float(kf_nis))
        self._kf_reject_window.append(0 if kf_accepted else 1)
        kf_updates_window = len(self._kf_reject_window)
        kf_rejected_window = int(sum(self._kf_reject_window))
        kf_reject_ratio = (
            float(kf_rejected_window) / float(kf_updates_window)
            if kf_updates_window > 0 else 0.0
        )
        result['kf_updates_window'] = int(kf_updates_window)
        result['kf_rejected_window'] = int(kf_rejected_window)
        result['kf_reject_ratio'] = round(kf_reject_ratio, 3)
        if len(self._kf_nis_window) > 0:
            result['kf_nis_median'] = round(float(np.median(self._kf_nis_window)), 3)

        # Compare raw/KF estimates against rover GPS reference (if available).
        if self._reference_lat is not None and self._reference_lon is not None:
            raw_err_m = self._haversine_m(
                raw_lat, raw_lon, self._reference_lat, self._reference_lon)
            kf_err_m = self._haversine_m(
                kf_lat, kf_lon, self._reference_lat, self._reference_lon)
            self._raw_err_window_m.append(raw_err_m)
            self._kf_err_window_m.append(kf_err_m)
            result['raw_error_m'] = round(raw_err_m, 1)
            result['kf_error_m'] = round(kf_err_m, 1)
            result['raw_error_med_m'] = round(float(np.median(self._raw_err_window_m)), 1)
            result['kf_error_med_m'] = round(float(np.median(self._kf_err_window_m)), 1)

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
            f'radius~{radius_km:.1f}km, '
            f'NIS={result.get("kf_nis", 0.0):.2f}, '
            f'accepted={result.get("kf_update_accepted", True)}')

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
            (kf_lat, kf_lon, kf_radius_km, nis, update_accepted)
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
            return lat, lon, max(1.0, 2.0 * meas_sigma_m / 1000.0), 0.0, True

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
        q_acc = max(0.01, self.kf_process_accel_mps2)  # m/s^2
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
        I = np.eye(4, dtype=np.float64)
        S_inv = np.linalg.inv(S)
        nis = float((y.T @ S_inv @ y)[0, 0])

        # Innovation gating: reject implausible measurement jumps.
        update_accepted = nis <= self.kf_innovation_gate
        if update_accepted:
            K = self._kf_P @ H.T @ S_inv
            self._kf_x = self._kf_x + (K @ y)
            # Joseph stabilized covariance update.
            I_KH = I - (K @ H)
            self._kf_P = I_KH @ self._kf_P @ I_KH.T + K @ R @ K.T
        else:
            # Prediction-only step when innovation is too large.
            pass

        fx = float(self._kf_x[0, 0])
        fy = float(self._kf_x[1, 0])
        kf_lat, kf_lon = self._kf_xy_to_latlon(fx, fy)

        pos_cov = self._kf_P[:2, :2]
        evals = np.linalg.eigvalsh(pos_cov)
        sigma_major_m = math.sqrt(max(float(evals[-1]), 0.0))
        kf_radius_km = max(1.0, (2.0 * sigma_major_m) / 1000.0)
        return kf_lat, kf_lon, kf_radius_km, nis, update_accepted

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
