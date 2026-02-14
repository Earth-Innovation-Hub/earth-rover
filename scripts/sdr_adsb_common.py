#!/usr/bin/env python3
"""
Shared ADS-B Components for SDR Decoder Nodes

Provides common classes used by both HydraSDR and RTL-SDR ADS-B decoders:
  - Aircraft:           Unified aircraft data model
  - AircraftTracker:    Thread-safe multi-aircraft tracker with eviction
  - ADSBIQDemodulator:  IQ signal -> ADS-B bit extraction (preamble + Manchester)
  - PyModeSDecoder:     pyModeS wrapper with proper CPR position decoding
  - ROS2 publishing helpers for NavSatFix and aircraft list

This module is installed alongside the ROS2 node scripts so they can
import it directly.

References:
  - ADS-B: https://en.wikipedia.org/wiki/Automatic_dependent_surveillance-broadcast
  - Mode S Extended Squitter type codes:
      TC  1-4:  Aircraft identification (callsign)
      TC  5-8:  Surface position
      TC  9-18: Airborne position (barometric altitude)
      TC 19:    Airborne velocity
      TC 20-22: Airborne position (GNSS altitude)
      TC 23-27: Reserved
      TC 28:    Aircraft status
      TC 29:    Target state and status
      TC 31:    Aircraft operation status
  - pyModeS CPR decoding requires BOTH an odd and even message to compute
    a globally unambiguous position.
"""

import time
import threading
import numpy as np
from collections import deque
from typing import Dict, Optional, Tuple, List

# Optional: pyModeS
try:
    import pyModeS as pms
    PYMODES_AVAILABLE = True
except ImportError:
    PYMODES_AVAILABLE = False

# Optional: scipy
try:
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from scipy import signal as scipy_signal
    SCIPY_AVAILABLE = True
except (ImportError, AttributeError, ValueError, Exception):
    SCIPY_AVAILABLE = False


# ============================================================================
# Aircraft Data Model
# ============================================================================

class Aircraft:
    """Unified aircraft data model for ADS-B tracking.

    All fields are stored individually (not as tuples) for clarity,
    safe comparison, and direct field access.
    """

    __slots__ = [
        'icao_address', 'callsign',
        'latitude', 'longitude', 'altitude',
        'speed', 'heading', 'vertical_rate',
        'squawk', 'category',
        'last_update', 'message_count',
        'on_ground',
    ]

    def __init__(self, icao_address: str):
        self.icao_address: str = icao_address
        self.callsign: Optional[str] = None

        # Position
        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.altitude: Optional[float] = None   # feet

        # Velocity
        self.speed: Optional[float] = None           # knots
        self.heading: Optional[float] = None         # degrees true
        self.vertical_rate: Optional[float] = None   # ft/min

        # Metadata
        self.squawk: Optional[str] = None
        self.category: Optional[str] = None
        self.on_ground: bool = False

        # Tracking
        self.last_update: float = time.time()
        self.message_count: int = 0

    def touch(self):
        """Update last-seen timestamp and increment message count."""
        self.last_update = time.time()
        self.message_count += 1

    def is_stale(self, timeout: float) -> bool:
        """Check if aircraft hasn't been seen within timeout seconds."""
        return (time.time() - self.last_update) > timeout

    def has_position(self) -> bool:
        """Check if a valid lat/lon position exists."""
        return self.latitude is not None and self.longitude is not None

    def summary(self) -> str:
        """One-line summary string for logging/publishing."""
        parts = [self.icao_address]
        if self.callsign:
            parts.append(f'({self.callsign})')
        if self.altitude is not None:
            parts.append(f'@{self.altitude:.0f}ft')
        if self.speed is not None:
            parts.append(f'{self.speed:.0f}kts')
        if self.heading is not None:
            parts.append(f'hdg {self.heading:.0f}°')
        return ' '.join(parts)


# ============================================================================
# Aircraft Tracker
# ============================================================================

class AircraftTracker:
    """Thread-safe multi-aircraft tracker with LRU eviction.

    Manages a dictionary of Aircraft objects keyed by ICAO address.
    When max_aircraft is reached, the least-recently-updated aircraft
    is evicted to make room for new ones.
    """

    def __init__(self, max_aircraft: int = 200, timeout: float = 60.0,
                 logger=None):
        self.aircraft: Dict[str, Aircraft] = {}
        self.max_aircraft = max_aircraft
        self.timeout = timeout
        self.lock = threading.Lock()
        self._log = logger   # ROS2 logger (optional)

    def get_or_create(self, icao: str) -> Tuple[Aircraft, bool]:
        """Get existing aircraft or create new one.

        Returns (aircraft, is_new).
        Must be called with self.lock held.
        """
        is_new = False
        if icao not in self.aircraft:
            # Evict oldest if at capacity
            if len(self.aircraft) >= self.max_aircraft:
                oldest_icao = min(
                    self.aircraft, key=lambda k: self.aircraft[k].last_update)
                del self.aircraft[oldest_icao]
                if self._log:
                    self._log.info(
                        f'[EVICT] Removed {oldest_icao} (capacity limit)')

            self.aircraft[icao] = Aircraft(icao)
            is_new = True
            if self._log:
                self._log.info(f'[NEW] Aircraft ICAO: {icao}')

        ac = self.aircraft[icao]
        ac.touch()
        return ac, is_new

    def cleanup_stale(self) -> List[str]:
        """Remove aircraft not seen within timeout. Returns removed ICAOs."""
        with self.lock:
            stale = [
                icao for icao, ac in self.aircraft.items()
                if ac.is_stale(self.timeout)
            ]
            for icao in stale:
                del self.aircraft[icao]
                if self._log:
                    self._log.info(f'[STALE] Removed {icao}')
        return stale

    def aircraft_list_string(self) -> str:
        """Return a summary string of all tracked aircraft."""
        with self.lock:
            entries = [ac.summary() for ac in self.aircraft.values()]
            return (
                f'Tracked Aircraft ({len(entries)}): '
                + ', '.join(entries)
            )

    @property
    def count(self) -> int:
        return len(self.aircraft)


# ============================================================================
# pyModeS Decoder (with correct CPR position handling)
# ============================================================================

class PyModeSDecoder:
    """Wraps pyModeS decoding with proper CPR position resolution.

    ADS-B airborne position messages use Compact Position Reporting (CPR).
    A single message only contains encoded lat/lon that is ambiguous.
    To get an unambiguous global position, you need BOTH an odd-flag
    and an even-flag message from the same aircraft, received within
    ~10 seconds of each other.

    This class maintains per-aircraft CPR buffers and only emits a
    position when both odd and even messages are available.
    """

    def __init__(self, logger=None):
        self._log = logger
        # Per-ICAO CPR buffer: {icao: {'even': (msg, time), 'odd': (msg, time)}}
        self._cpr_buffer: Dict[str, dict] = {}
        # Max age for CPR message pair (seconds)
        self._cpr_max_age = 10.0

    def decode(self, hex_msg: str, tracker: AircraftTracker) -> Optional[Aircraft]:
        """Decode a 28-char hex ADS-B message and update the tracker.

        Returns the updated Aircraft if decoding succeeded, else None.
        """
        if not PYMODES_AVAILABLE:
            return None

        try:
            if len(hex_msg) != 28:
                return None

            df = pms.df(hex_msg)
            if df != 17:  # Only Extended Squitter
                return None

            icao = pms.icao(hex_msg)
            tc = pms.typecode(hex_msg)

            if not icao or not tc:
                return None

            with tracker.lock:
                ac, is_new = tracker.get_or_create(icao)

                # TC 1-4: Aircraft identification
                if 1 <= tc <= 4:
                    callsign = pms.callsign(hex_msg)
                    if callsign:
                        callsign = callsign.strip()
                        if callsign and ac.callsign != callsign:
                            if self._log:
                                self._log.info(
                                    f'[ID] {icao}: {callsign}')
                            ac.callsign = callsign

                # TC 5-8: Surface position
                elif 5 <= tc <= 8:
                    ac.on_ground = True
                    self._decode_surface_position(hex_msg, icao, ac)

                # TC 9-18: Airborne position (barometric altitude)
                elif 9 <= tc <= 18:
                    ac.on_ground = False
                    alt = pms.altitude(hex_msg)
                    if alt is not None:
                        ac.altitude = float(alt)
                    self._decode_airborne_position(hex_msg, icao, ac)

                # TC 19: Airborne velocity
                elif tc == 19:
                    try:
                        velocity = pms.velocity(hex_msg)
                        if velocity:
                            spd, hdg, vrate, _ = velocity
                            if spd is not None:
                                ac.speed = float(spd)
                                if self._log:
                                    self._log.info(
                                        f'[VEL] {icao}: {spd:.0f}kts '
                                        f'hdg={hdg:.0f}° vrate={vrate:.0f}fpm')
                            if hdg is not None:
                                ac.heading = float(hdg)
                            if vrate is not None:
                                ac.vertical_rate = float(vrate)
                    except Exception:
                        pass

                # TC 20-22: Airborne position (GNSS altitude)
                elif 20 <= tc <= 22:
                    ac.on_ground = False
                    alt = pms.altitude(hex_msg)
                    if alt is not None:
                        ac.altitude = float(alt)
                    self._decode_airborne_position(hex_msg, icao, ac)

                return ac

        except Exception as e:
            if self._log:
                self._log.debug(f'Decode error: {e}')
            return None

    def _decode_airborne_position(self, hex_msg: str, icao: str,
                                   ac: Aircraft):
        """Decode airborne position using CPR (requires odd+even pair)."""
        try:
            oe = pms.oe_flag(hex_msg)  # 0=even, 1=odd
            t = time.time()

            if icao not in self._cpr_buffer:
                self._cpr_buffer[icao] = {}

            key = 'odd' if oe else 'even'
            self._cpr_buffer[icao][key] = (hex_msg, t)

            buf = self._cpr_buffer[icao]
            if 'odd' in buf and 'even' in buf:
                msg_odd, t_odd = buf['odd']
                msg_even, t_even = buf['even']

                # Both messages must be recent
                if abs(t_odd - t_even) < self._cpr_max_age:
                    try:
                        lat, lon = pms.position(
                            msg_even, msg_odd, t_even, t_odd)
                        if lat is not None and lon is not None:
                            # Sanity check: valid coordinate range
                            if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                                ac.latitude = float(lat)
                                ac.longitude = float(lon)
                                if self._log:
                                    self._log.info(
                                        f'[POS] {icao}: ({lat:.6f}, '
                                        f'{lon:.6f}) alt={ac.altitude}ft')
                    except Exception as e:
                        if self._log:
                            self._log.debug(
                                f'CPR decode failed for {icao}: {e}')
                else:
                    # Pair too old, discard the older one
                    if t_odd < t_even:
                        del buf['odd']
                    else:
                        del buf['even']

        except Exception as e:
            if self._log:
                self._log.debug(f'Airborne position error for {icao}: {e}')

    def _decode_surface_position(self, hex_msg: str, icao: str,
                                  ac: Aircraft):
        """Decode surface position. Surface messages also use CPR."""
        # Surface position decoding requires a reference position
        # (receiver location) for unambiguous resolution.
        # For now, treat similarly to airborne but mark on_ground.
        try:
            alt = pms.altitude(hex_msg)
            if alt is not None:
                ac.altitude = float(alt)
            else:
                ac.altitude = 0.0
            # CPR decoding for surface is the same odd/even approach
            self._decode_airborne_position(hex_msg, icao, ac)
        except Exception:
            pass

    def prune_cpr_buffer(self, max_age: float = 30.0):
        """Remove stale CPR entries to prevent memory growth."""
        now = time.time()
        stale_icaos = []
        for icao, buf in self._cpr_buffer.items():
            newest = 0.0
            for key in ('odd', 'even'):
                if key in buf:
                    newest = max(newest, buf[key][1])
            if now - newest > max_age:
                stale_icaos.append(icao)
        for icao in stale_icaos:
            del self._cpr_buffer[icao]


# ============================================================================
# IQ Demodulator for ADS-B
# ============================================================================

class ADSBIQDemodulator:
    """Demodulates ADS-B signals from raw IQ samples.

    Performs:
      1. Power envelope extraction
      2. Optional low-pass filtering (scipy)
      3. Preamble detection (8us high-low-high-low-low pattern)
      4. Manchester bit extraction (112 bits)
      5. Conversion to 28-char hex string

    Works with any sample rate; adapts preamble/bit timing automatically.
    """

    def __init__(self, sample_rate: float = 2.4e6, logger=None):
        self.sample_rate = sample_rate
        self._log = logger
        self._samples_per_us = sample_rate / 1e6

    def demodulate(self, iq_array: np.ndarray) -> List[str]:
        """Extract ADS-B hex messages from complex IQ array.

        Returns a list of 28-character hex strings (may be empty).
        """
        messages = []

        min_samples = int(self.sample_rate * 0.000120)  # 120us message
        if len(iq_array) < min_samples:
            return messages

        power = np.abs(iq_array)

        # Optional low-pass filter
        if SCIPY_AVAILABLE and len(power) > 100:
            try:
                b, a = scipy_signal.butter(3, 0.1, 'low')
                power = scipy_signal.filtfilt(b, a, power)
            except Exception:
                pass

        # Normalize
        peak = np.max(power)
        if peak == 0:
            return messages
        normalized = power / peak

        threshold = np.mean(normalized) + 0.3 * (1.0 - np.mean(normalized))

        spu = self._samples_per_us
        preamble_len = int(16 * spu)
        msg_samples = int(112 * spu)

        # Search for preambles across the buffer
        search_limit = len(normalized) - preamble_len - msg_samples
        idx = 0

        while idx < search_limit:
            # Quick energy check for preamble region
            preamble_region = normalized[idx:idx + preamble_len]
            if np.mean(preamble_region) < threshold * 0.4:
                idx += int(spu)
                continue

            # Check preamble pattern: high-low-high-low-low-low
            if self._check_preamble(normalized, idx, threshold):
                # Extract 112 message bits
                msg_start = idx + preamble_len
                hex_msg = self._extract_message(normalized, msg_start)

                if hex_msg:
                    messages.append(hex_msg)
                    # Skip past this message
                    idx = msg_start + msg_samples
                    continue

            idx += int(spu)

        return messages

    def _check_preamble(self, normalized: np.ndarray, start: int,
                        threshold: float) -> bool:
        """Check for ADS-B preamble at the given position.

        Preamble pattern (in microseconds):
          0-1us:   HIGH  (pulse 1)
          1-2us:   LOW
          2-3us:   HIGH  (pulse 2)
          3-4.5us: LOW
          4.5-5.5us: HIGH (pulse 3)
          5.5-8us: LOW
          (8us total preamble)
        """
        spu = self._samples_per_us

        # Define expected high/low segments as (start_us, end_us, expect_high)
        segments = [
            (0.0, 1.0, True),    # Pulse 1
            (1.0, 2.0, False),   # Gap
            (2.0, 3.0, True),    # Pulse 2
            (3.0, 4.5, False),   # Gap
            (4.5, 5.5, True),    # Pulse 3
            (5.5, 8.0, False),   # Gap
        ]

        for start_us, end_us, expect_high in segments:
            seg_start = start + int(start_us * spu)
            seg_end = start + int(end_us * spu)

            if seg_end > len(normalized):
                return False

            seg_mean = np.mean(normalized[seg_start:seg_end])

            if expect_high and seg_mean < threshold:
                return False
            if not expect_high and seg_mean > threshold * 0.7:
                return False

        return True

    def _extract_message(self, normalized: np.ndarray,
                         start: int) -> Optional[str]:
        """Extract 112 bits using Manchester decoding and convert to hex."""
        spu = self._samples_per_us
        bit_dur = int(spu)
        bits = []

        for i in range(112):
            bs = start + i * bit_dur
            be = bs + bit_dur

            if be > len(normalized):
                return None

            seg = normalized[bs:be]
            mid = len(seg) // 2

            if mid == 0:
                return None

            first_half = np.mean(seg[:mid])
            second_half = np.mean(seg[mid:])

            # Manchester: high-to-low = 0, low-to-high = 1
            if first_half > second_half:
                bits.append(0)
            else:
                bits.append(1)

        if len(bits) != 112:
            return None

        # Convert to hex
        hex_chars = []
        for i in range(0, 112, 4):
            nibble = bits[i:i + 4]
            val = int(''.join(str(b) for b in nibble), 2)
            hex_chars.append(f'{val:X}')

        return ''.join(hex_chars)


# ============================================================================
# ROS2 Publishing Helpers
# ============================================================================

def publish_aircraft_navsatfix(ac: Aircraft, clock, publisher):
    """Publish an Aircraft as a sensor_msgs/NavSatFix message.

    Args:
        ac: Aircraft with position data
        clock: ROS2 clock (node.get_clock())
        publisher: ROS2 publisher for NavSatFix
    """
    if not ac.has_position():
        return

    # Import here to avoid hard dependency at module level
    from sensor_msgs.msg import NavSatFix

    msg = NavSatFix()
    msg.header.stamp = clock.now().to_msg()
    msg.header.frame_id = f'aircraft_{ac.icao_address}'
    msg.latitude = ac.latitude
    msg.longitude = ac.longitude
    msg.altitude = ac.altitude if ac.altitude is not None else 0.0
    msg.status.status = 1   # STATUS_FIX
    msg.status.service = 1  # SERVICE_GPS

    publisher.publish(msg)


def publish_aircraft_list_string(tracker: AircraftTracker, publisher):
    """Publish the tracker's aircraft list as a std_msgs/String.

    Args:
        tracker: AircraftTracker instance
        publisher: ROS2 publisher for String
    """
    from std_msgs.msg import String

    msg = String()
    msg.data = tracker.aircraft_list_string()
    publisher.publish(msg)
