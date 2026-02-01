#!/usr/bin/env python3
"""
ADS-B Decoder Node for HydraSDR

Decodes ADS-B (Automatic Dependent Surveillance-Broadcast) messages from
aircraft using the HydraSDR RFOne. ADS-B operates at 1090 MHz and uses
Mode S Extended Squitter protocol.

This node:
- Subscribes to IQ samples from HydraSDR node
- Demodulates and decodes ADS-B messages
- Publishes aircraft position, velocity, and identification
- Tracks multiple aircraft simultaneously

Topics Subscribed:
  ~/iq_samples (std_msgs/Float32MultiArray): IQ samples from SDR

Topics Published:
  ~/aircraft (adsb_msgs/Aircraft): Decoded aircraft data
  ~/aircraft_list (adsb_msgs/AircraftList): List of all tracked aircraft
  ~/messages (std_msgs/String): Raw decoded messages (debug)

Parameters:
  iq_topic (string): Topic for IQ samples (default: /hydra_sdr/hydra_sdr_node/iq_samples)
  sample_rate (float): Sample rate in Hz (default: 2.5e6)
  center_frequency (float): Center frequency in Hz (default: 1090e6 for ADS-B)
  enable_decoding (bool): Enable message decoding (default: true)
  publish_raw_messages (bool): Publish raw decoded messages (default: false)
  max_aircraft (int): Maximum number of aircraft to track (default: 100)
  aircraft_timeout (float): Timeout for removing stale aircraft (default: 60.0 seconds)

Dependencies:
  - pyModeS: pip install pyModeS
  - numpy
  - scipy (for signal processing)

References:
  - ADS-B: https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
  - Mode S: https://en.wikipedia.org/wiki/Mode_S
  - pyModeS: https://github.com/junzis/pyModeS
"""

import numpy as np
import time
from collections import defaultdict, deque
from typing import Dict, Optional, Tuple
import threading

# Try to import scipy for signal processing
SCIPY_AVAILABLE = False
try:
    import scipy
    from scipy import signal
    SCIPY_AVAILABLE = True
except (ImportError, AttributeError, ValueError) as e:
    SCIPY_AVAILABLE = False
    # Silently handle scipy import errors (version incompatibilities, etc.)
    pass

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import NavSatFix

# Try to import pyModeS for ADS-B decoding
try:
    import pyModeS as pms
    PYMODES_AVAILABLE = True
except ImportError:
    PYMODES_AVAILABLE = False
    print("Warning: pyModeS not installed. Install with: pip install pyModeS")
    print("ADS-B decoding will be disabled.")


class Aircraft:
    """Represents a tracked aircraft."""
    
    def __init__(self, icao_address: str):
        self.icao_address = icao_address
        self.callsign = None
        self.position = None  # (lat, lon, alt)
        self.velocity = None  # (speed, heading, vertical_rate)
        self.last_update = time.time()
        self.message_count = 0
        self.aircraft_type = None
        self.category = None
    
    def update(self):
        """Update last seen timestamp."""
        self.last_update = time.time()
        self.message_count += 1
    
    def is_stale(self, timeout: float) -> bool:
        """Check if aircraft data is stale."""
        return (time.time() - self.last_update) > timeout


class ADSBDecoderNode(Node):
    """ROS2 node for decoding ADS-B messages from SDR IQ samples."""
    
    def __init__(self):
        super().__init__('adsb_decoder_node')
        
        # ====================================================================
        # Parameters
        # ====================================================================
        
        self.declare_parameter('iq_topic', '/hydra_sdr/hydra_sdr_node/iq_samples')
        self.declare_parameter('sample_rate', 2.5e6)
        self.declare_parameter('center_frequency', 1090e6)  # ADS-B frequency
        self.declare_parameter('enable_decoding', True)
        self.declare_parameter('publish_raw_messages', False)
        self.declare_parameter('max_aircraft', 100)
        self.declare_parameter('aircraft_timeout', 60.0)
        self.declare_parameter('demod_threshold', 0.5)  # Demodulation threshold
        self.declare_parameter('debug', False)  # Enable debug messages
        
        # Get parameters
        iq_topic = self.get_parameter('iq_topic').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.center_freq = self.get_parameter('center_frequency').value
        self.enable_decoding = self.get_parameter('enable_decoding').value
        self.publish_raw = self.get_parameter('publish_raw_messages').value
        self.max_aircraft = self.get_parameter('max_aircraft').value
        self.timeout = self.get_parameter('aircraft_timeout').value
        self.demod_threshold = self.get_parameter('demod_threshold').value
        self.debug = self.get_parameter('debug').value
        
        # ====================================================================
        # State
        # ====================================================================
        
        self.aircraft: Dict[str, Aircraft] = {}
        self.raw_message_buffer = deque(maxlen=1000)
        self.lock = threading.Lock()
        
        # Signal processing buffers
        self.iq_buffer = deque(maxlen=int(self.sample_rate * 0.1))  # 100ms buffer
        self.position_buffer = {}  # For combining odd/even position messages
        
        # ADS-B message parameters
        self.adsb_bit_rate = 1e6  # 1 Mbps
        self.adsb_preamble = [1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0]
        
        # ====================================================================
        # Publishers
        # ====================================================================
        
        # Create custom message types or use standard messages
        # For now, we'll publish as NavSatFix and custom topics
        self.aircraft_pub = self.create_publisher(NavSatFix, '~/aircraft', 10)
        self.aircraft_list_pub = self.create_publisher(String, '~/aircraft_list', 10)
        
        if self.publish_raw:
            self.raw_msg_pub = self.create_publisher(String, '~/messages', 10)
        
        # ====================================================================
        # Subscribers
        # ====================================================================
        
        if PYMODES_AVAILABLE and self.enable_decoding:
            self.iq_sub = self.create_subscription(
                Float32MultiArray,
                iq_topic,
                self.iq_callback,
                10
            )
        else:
            self.get_logger().warn('ADS-B decoding disabled - pyModeS not available')
        
        # ====================================================================
        # Timers
        # ====================================================================
        
        # Periodic cleanup of stale aircraft
        self.cleanup_timer = self.create_timer(10.0, self.cleanup_stale_aircraft)
        
        # Periodic publication of aircraft list
        self.publish_timer = self.create_timer(1.0, self.publish_aircraft_list)
        
        self.get_logger().info('ADS-B Decoder Node initialized')
        self.get_logger().info(f'  IQ Topic: {iq_topic}')
        self.get_logger().info(f'  Sample Rate: {self.sample_rate/1e6:.1f} MSPS')
        self.get_logger().info(f'  Center Frequency: {self.center_freq/1e6:.3f} MHz')
        self.get_logger().info(f'  Decoding: {"Enabled" if (PYMODES_AVAILABLE and self.enable_decoding) else "Disabled"}')
        self.get_logger().info(f'  Debug Mode: {"Enabled" if self.debug else "Disabled"}')
        if not SCIPY_AVAILABLE:
            self.get_logger().warn('  Scipy not available - filtering disabled (may affect decoding performance)')
        
        # Verify frequency is correct for ADS-B
        if abs(self.center_freq - 1090e6) > 1e6:  # More than 1 MHz off
            self.get_logger().warn(f'  WARNING: Center frequency is {self.center_freq/1e6:.3f} MHz, expected 1090 MHz for ADS-B!')
    
    def iq_callback(self, msg):
        """Process incoming IQ samples."""
        try:
            # Parse IQ data (interleaved I and Q)
            iq_data = np.array(msg.data, dtype=np.float32)
            
            if len(iq_data) < 2:
                return
            
            # Separate I and Q
            i_samples = iq_data[0::2]
            q_samples = iq_data[1::2]
            
            # Form complex IQ
            iq = i_samples + 1j * q_samples
            
            # Add to buffer
            self.iq_buffer.extend(iq)
            
            # Process buffer for ADS-B messages
            if len(self.iq_buffer) > int(self.sample_rate * 0.05):  # At least 50ms
                self.process_adsb_signals()
            
            # Debug: log IQ reception
            if self.debug and hasattr(self, '_last_iq_log_time'):
                now = time.time()
                if now - self._last_iq_log_time > 5.0:  # Every 5 seconds
                    self.get_logger().info(f'[DEBUG] Received IQ samples: {len(iq)} samples, buffer size: {len(self.iq_buffer)}')
                    self._last_iq_log_time = now
            elif self.debug:
                self._last_iq_log_time = time.time()
        
        except Exception as e:
            self.get_logger().error(f'Error in IQ callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def process_adsb_signals(self):
        """Process IQ buffer to extract and decode ADS-B messages."""
        if not PYMODES_AVAILABLE:
            return
        
        try:
            # Convert buffer to numpy array
            iq_array = np.array(list(self.iq_buffer))
            
            if len(iq_array) < int(self.sample_rate * 0.000120):  # Need at least 120us (message length)
                return
            
            # Demodulate PPM (Pulse Position Modulation)
            # ADS-B uses Manchester-encoded PPM
            power = np.abs(iq_array)
            
            # Low-pass filter to smooth the signal (optional, improves detection)
            if SCIPY_AVAILABLE and len(power) > 100:
                try:
                    b, a = signal.butter(3, 0.1, 'low')
                    power = signal.filtfilt(b, a, power)
                except Exception as e:
                    # If filtering fails, continue without it
                    self.get_logger().debug(f'Filtering failed: {e}, continuing without filter')
            
            # Detect preamble pattern
            # ADS-B preamble: 1010000101000000 (8us high, 0.5us low, 0.5us high, 1.5us low, 0.5us high, 4.5us low)
            # In samples at 2.5 MSPS: [20, 1, 1, 3, 1, 11] samples
            preamble_pattern = self.detect_preamble(power)
            
            if self.debug and preamble_pattern is not None:
                self.get_logger().info(f'[DEBUG] Preamble detected at sample index: {preamble_pattern}')
            
            if preamble_pattern is not None:
                # Extract message bits starting after preamble
                message_bits = self.extract_message_bits(power, preamble_pattern)
                
                if self.debug and message_bits is not None:
                    self.get_logger().info(f'[DEBUG] Extracted {len(message_bits)} message bits')
                
                if message_bits and len(message_bits) >= 112:  # ADS-B message is 112 bits
                    # Convert bits to hex string
                    hex_message = self.bits_to_hex(message_bits[:112])
                    
                    if hex_message:
                        if self.debug:
                            self.get_logger().info(f'[DEBUG] Decoding message: {hex_message[:14]}...')
                        
                        # Decode message
                        decoded = self.decode_adsb_message(hex_message)
                        if decoded:
                            icao = decoded.get('icao', 'UNKNOWN')
                            self.get_logger().info(f'[DETECTED] Aircraft: {icao} | Type: {decoded.get("type_code", "N/A")} | Callsign: {decoded.get("callsign", "N/A")}')
                            self.update_aircraft(decoded)
                            
                            if self.publish_raw:
                                msg = String()
                                msg.data = f"{icao}: {hex_message}"
                                self.raw_msg_pub.publish(msg)
                        elif self.debug:
                            self.get_logger().debug(f'[DEBUG] Failed to decode message: {hex_message}')
            
            # Keep some overlap for messages that span buffer boundaries
            # Only clear if buffer is getting too large
            if len(self.iq_buffer) > int(self.sample_rate * 0.1):
                # Keep last 20ms
                keep_samples = int(self.sample_rate * 0.02)
                buffer_list = list(self.iq_buffer)
                self.iq_buffer.clear()
                self.iq_buffer.extend(buffer_list[-keep_samples:])
            
        except Exception as e:
            self.get_logger().error(f'Error processing ADS-B signals: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def detect_preamble(self, power: np.ndarray) -> Optional[int]:
        """Detect ADS-B preamble pattern in power signal."""
        # Simplified preamble detection
        # Look for characteristic pattern: high-low-high-low-low pattern
        
        # Normalize power
        if np.max(power) == 0:
            return None
        
        normalized = power / np.max(power)
        
        # Threshold for high/low detection
        threshold = np.mean(normalized) + 0.3 * (np.max(normalized) - np.mean(normalized))
        
        # Sample rate dependent: at 2.5 MSPS, 1 bit = 1us = 2.5 samples
        samples_per_bit = self.sample_rate / 1e6  # samples per microsecond
        
        # Preamble pattern in microseconds: [8, 0.5, 0.5, 1.5, 0.5, 4.5]
        # Convert to samples
        preamble_samples = [int(x * samples_per_bit) for x in [8, 0.5, 0.5, 1.5, 0.5, 4.5]]
        
        # Search for pattern
        search_window = int(20 * samples_per_bit)  # Search first 20us
        
        for start in range(min(search_window, len(normalized) - sum(preamble_samples))):
            # Check if pattern matches
            idx = start
            matches = True
            
            for i, duration in enumerate(preamble_samples):
                if idx + duration > len(normalized):
                    matches = False
                    break
                
                segment = normalized[idx:idx+duration]
                expected_high = (i % 2 == 0)  # Even indices should be high
                
                if expected_high:
                    if np.mean(segment) < threshold:
                        matches = False
                        break
                else:
                    if np.mean(segment) > threshold * 0.7:
                        matches = False
                        break
                
                idx += duration
            
            if matches:
                return start + sum(preamble_samples[:6])  # Return position after preamble
        
        return None
    
    def extract_message_bits(self, power: np.ndarray, start_idx: int) -> Optional[np.ndarray]:
        """Extract message bits from power signal using Manchester decoding."""
        if start_idx >= len(power):
            return None
        
        # Sample rate dependent
        samples_per_bit = self.sample_rate / 1e6  # samples per microsecond
        bit_duration = int(samples_per_bit)
        
        # Extract 112 bits (ADS-B message length)
        message_length = 112
        bits = []
        
        for i in range(message_length):
            bit_start = start_idx + i * bit_duration
            bit_end = bit_start + bit_duration
            
            if bit_end > len(power):
                break
            
            # Manchester decoding: transition in middle of bit period
            # High-to-low = 0, Low-to-high = 1
            bit_segment = power[bit_start:bit_end]
            mid_point = len(bit_segment) // 2
            
            first_half = np.mean(bit_segment[:mid_point])
            second_half = np.mean(bit_segment[mid_point:])
            
            # Determine bit value based on transition
            threshold = np.mean(power)
            if first_half > threshold and second_half < threshold:
                bits.append(0)  # High-to-low transition
            elif first_half < threshold and second_half > threshold:
                bits.append(1)  # Low-to-high transition
            else:
                # No clear transition, use majority
                bits.append(1 if np.mean(bit_segment) > threshold else 0)
        
        return np.array(bits) if len(bits) == message_length else None
    
    def bits_to_hex(self, bits: np.ndarray) -> Optional[str]:
        """Convert bit array to hex string."""
        if len(bits) % 4 != 0:
            return None
        
        hex_chars = []
        for i in range(0, len(bits), 4):
            nibble = bits[i:i+4]
            value = int(''.join(map(str, nibble)), 2)
            hex_chars.append(f'{value:X}')
        
        return ''.join(hex_chars)
    
    def decode_adsb_message(self, message: str) -> Optional[Dict]:
        """Decode an ADS-B message using pyModeS."""
        if not PYMODES_AVAILABLE:
            return None
        
        try:
            # Parse the message (hex string)
            if len(message) != 28:  # ADS-B messages are 112 bits = 28 hex chars
                return None
            
            # Decode using pyModeS
            df = pms.df(message)  # Downlink format
            
            if df == 17:  # Extended Squitter (ADS-B)
                icao = pms.icao(message)
                tc = pms.typecode(message)
                
                result = {
                    'icao': icao,
                    'type_code': tc,
                    'raw': message
                }
                
                # Decode based on type code
                if 1 <= tc <= 4:  # Aircraft identification
                    callsign = pms.callsign(message)
                    result['callsign'] = callsign
                
                elif 9 <= tc <= 18:  # Surface position
                    # Surface position messages
                    pass
                
                elif 5 <= tc <= 8:  # Airborne position (odd/even)
                    # Position messages (need both odd and even)
                    # Store for later combination
                    if 'position_buffer' not in self.__dict__:
                        self.position_buffer = {}
                    
                    odd = pms.oe_flag(message)
                    lat, lon = pms.position(message, self.position_buffer.get(icao, {}))
                    
                    if lat and lon:
                        alt = pms.altitude(message)
                        result['position'] = (lat, lon, alt if alt else 0)
                        
                        # Store in buffer for next message
                        if icao not in self.position_buffer:
                            self.position_buffer[icao] = {}
                        self.position_buffer[icao][odd] = (lat, lon)
                
                elif 19 <= tc <= 22:  # Airborne velocity
                    # Velocity messages
                    spd, hdg, roc, tag = pms.velocity(message)
                    result['speed'] = spd
                    result['heading'] = hdg
                    result['vertical_rate'] = roc
                
                return result
            
            return None
        
        except Exception as e:
            self.get_logger().debug(f'Error decoding message: {e}')
            return None
    
    def update_aircraft(self, decoded: Dict):
        """Update aircraft data from decoded message."""
        with self.lock:
            icao = decoded.get('icao')
            if not icao:
                if self.debug:
                    self.get_logger().debug('[DEBUG] No ICAO address in decoded message')
                return
            
            # Get or create aircraft
            if icao not in self.aircraft:
                if len(self.aircraft) >= self.max_aircraft:
                    # Remove oldest aircraft
                    oldest = min(self.aircraft.items(), key=lambda x: x[1].last_update)
                    removed_icao = oldest[0]
                    del self.aircraft[removed_icao]
                    if self.debug:
                        self.get_logger().info(f'[DEBUG] Removed oldest aircraft {removed_icao} (max limit reached)')
                
                self.aircraft[icao] = Aircraft(icao)
                self.get_logger().info(f'[NEW AIRCRAFT] ICAO: {icao}')
            
            aircraft = self.aircraft[icao]
            aircraft.update()
            
            # Update aircraft data
            if 'callsign' in decoded and decoded['callsign']:
                if aircraft.callsign != decoded['callsign']:
                    self.get_logger().info(f'[UPDATE] {icao}: Callsign = {decoded["callsign"]}')
                aircraft.callsign = decoded['callsign']
            
            if 'position' in decoded and decoded['position']:
                lat, lon, alt = decoded['position']
                if aircraft.position != decoded['position']:
                    self.get_logger().info(f'[UPDATE] {icao}: Position = ({lat:.6f}, {lon:.6f}) Alt = {alt:.0f}ft')
                aircraft.position = decoded['position']
            
            if 'speed' in decoded:
                speed = decoded.get('speed', 0)
                heading = decoded.get('heading', 0)
                vrate = decoded.get('vertical_rate', 0)
                if aircraft.velocity != (speed, heading, vrate):
                    self.get_logger().info(f'[UPDATE] {icao}: Speed = {speed:.0f} kts, Heading = {heading:.0f}°, VRate = {vrate:.0f} fpm')
                aircraft.velocity = (speed, heading, vrate)
            
            # Publish individual aircraft update
            self.publish_aircraft(aircraft)
    
    def publish_aircraft(self, aircraft: Aircraft):
        """Publish aircraft data as NavSatFix message."""
        if aircraft.position is None:
            return
        
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'aircraft_{aircraft.icao_address}'
        msg.latitude = aircraft.position[0]
        msg.longitude = aircraft.position[1]
        msg.altitude = aircraft.position[2]
        msg.status.status = 1  # Fix
        msg.status.service = 1  # GPS
        
        self.aircraft_pub.publish(msg)
    
    def publish_aircraft_list(self):
        """Publish list of all tracked aircraft."""
        with self.lock:
            aircraft_list = []
            for icao, aircraft in self.aircraft.items():
                info = f"{icao}"
                if aircraft.callsign:
                    info += f" ({aircraft.callsign})"
                if aircraft.position:
                    info += f" @ {aircraft.position[2]:.0f}ft"
                aircraft_list.append(info)
            
            msg = String()
            msg.data = f"Tracked Aircraft ({len(aircraft_list)}): " + ", ".join(aircraft_list)
            self.aircraft_list_pub.publish(msg)
    
    def cleanup_stale_aircraft(self):
        """Remove aircraft that haven't been seen recently."""
        with self.lock:
            stale = [icao for icao, ac in self.aircraft.items() if ac.is_stale(self.timeout)]
            for icao in stale:
                del self.aircraft[icao]
                self.get_logger().info(f'Removed stale aircraft: {icao}')


def main(args=None):
    """Main entry point."""
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

