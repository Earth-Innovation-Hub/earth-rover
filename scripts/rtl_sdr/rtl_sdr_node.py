#!/usr/bin/env python3
"""
RTL-SDR V4 ROS2 Node

Wraps the rtl-sdr command-line tools (rtl_sdr, rtl_test, rtl_biast) to provide
ROS2 integration for the RTL-SDR Blog V4 software-defined radio.

RTL-SDR V4 Hardware:
  - Chipset: RTL2832U + R828D tuner
  - Frequency range: 24 MHz - 1766 MHz (with HF direct sampling: 500 kHz - 28.8 MHz)
  - Sample rate: 225 kHz - 3.2 MSPS (2.4 MSPS recommended for stability)
  - ADC resolution: 8-bit unsigned IQ
  - Features: Bias-T (4.5V), direct sampling for HF, PPM correction
  - Gain: 0 - 49.6 dB (discrete steps via R828D tuner)

This node:
  - Detects and manages RTL-SDR V4 devices
  - Streams IQ samples using rtl_sdr (piped to stdout)
  - Converts 8-bit unsigned IQ to float32 for ROS2 publication
  - Publishes raw IQ data, spectrum FFT, and device status
  - Provides services for configuration changes
  - Supports dynamic reconfigure for frequency, gain, sample rate

Topics Published:
  ~/iq_samples (std_msgs/Float32MultiArray): Raw IQ samples (float32, interleaved)
  ~/spectrum   (std_msgs/Float32MultiArray): FFT power spectrum in dB
  ~/device_status   (std_msgs/String): Device connection and health status
  ~/signal_strength (std_msgs/Float32): Peak received signal strength (dB)

Parameters:
  auto_connect (bool): Automatically connect to first available device [true]
  auto_stream (bool): Automatically start streaming on startup [false]
  center_frequency (float): RF center frequency in Hz [433.0e6]
  sample_rate (float): Sample rate in Hz [2.4e6]
  gain (float): Tuner gain in dB, or -1 for auto-gain [-1]
  bias_tee (bool): Enable 4.5V bias tee for active antenna/LNA [false]
  ppm_correction (int): Frequency correction in PPM [0]
  direct_sampling (int): 0=disabled, 1=I-branch, 2=Q-branch [0]
  device_index (int): RTL-SDR device index for multi-dongle setups [0]
  agc_mode (bool): Enable RTL2832U internal AGC [false]
  publish_raw_iq (bool): Publish raw IQ samples [true]
  publish_spectrum (bool): Compute and publish FFT spectrum [true]
  fft_size (int): FFT size for spectrum computation [1024]
  spectrum_averaging (int): Number of spectra to average [4]

Dependencies:
  - rtl-sdr tools installed (rtl_sdr, rtl_test, rtl_biast)
  - Recommended: rtl-sdr-blog fork for V4 support
    https://github.com/rtlsdrblog/rtl-sdr-blog
"""

import os
import sys
import subprocess
import threading
import time
import numpy as np
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor, ParameterType, FloatingPointRange, IntegerRange,
    SetParametersResult
)
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
from std_srvs.srv import SetBool, Trigger


# RTL-SDR V4 valid gain values (R828D tuner, in tenths of dB)
# These are the discrete gain steps supported by the hardware
RTLSDR_GAIN_VALUES = [
    0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7,
    16.6, 19.7, 20.7, 22.9, 25.4, 28.0, 29.7, 32.8, 33.8,
    36.4, 37.2, 38.6, 40.2, 42.1, 43.4, 43.9, 44.5, 48.0, 49.6
]


class RTLSDRNode(Node):
    """ROS2 node for RTL-SDR Blog V4 software-defined radio."""

    def __init__(self):
        super().__init__('rtl_sdr_node')

        # ====================================================================
        # Declare Parameters with Descriptors for Dynamic Reconfigure
        # ====================================================================

        # Frequency range: 24-1766 MHz (R828D tuner range)
        freq_range = FloatingPointRange()
        freq_range.from_value = 24.0e6
        freq_range.to_value = 1766.0e6
        freq_range.step = 1.0e3

        # Sample rate: 225.001 kHz to 3.2 MSPS (2.4 recommended)
        sample_rate_range = FloatingPointRange()
        sample_rate_range.from_value = 225.001e3
        sample_rate_range.to_value = 3.2e6
        sample_rate_range.step = 1.0e3

        # Gain: -1 (auto) or 0 to 49.6 dB
        gain_range = FloatingPointRange()
        gain_range.from_value = -1.0
        gain_range.to_value = 49.6
        gain_range.step = 0.1

        # PPM correction
        ppm_range = IntegerRange()
        ppm_range.from_value = -500
        ppm_range.to_value = 500

        # Direct sampling mode
        ds_range = IntegerRange()
        ds_range.from_value = 0
        ds_range.to_value = 2

        # Device index
        dev_range = IntegerRange()
        dev_range.from_value = 0
        dev_range.to_value = 15

        fft_size_range = IntegerRange()
        fft_size_range.from_value = 256
        fft_size_range.to_value = 8192
        fft_size_range.step = 256

        self.declare_parameter('auto_connect', True,
            ParameterDescriptor(description='Automatically connect to device on startup'))
        self.declare_parameter('auto_stream', False,
            ParameterDescriptor(description='Automatically start streaming on startup'))

        self.declare_parameter('center_frequency', 433.0e6,
            ParameterDescriptor(
                description='RF center frequency in Hz (24-1766 MHz)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[freq_range]
            ))

        self.declare_parameter('sample_rate', 2.4e6,
            ParameterDescriptor(
                description='Sample rate in Hz (225 kHz - 3.2 MSPS, 2.4 MSPS recommended)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[sample_rate_range]
            ))

        self.declare_parameter('gain', -1.0,
            ParameterDescriptor(
                description='Tuner gain in dB (-1 for auto-gain, 0-49.6 for manual)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[gain_range]
            ))

        self.declare_parameter('bias_tee', False,
            ParameterDescriptor(description='Enable 4.5V bias tee for active antenna/LNA'))

        self.declare_parameter('ppm_correction', 0,
            ParameterDescriptor(
                description='Frequency correction in PPM (-500 to 500)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[ppm_range]
            ))

        self.declare_parameter('direct_sampling', 0,
            ParameterDescriptor(
                description='Direct sampling mode: 0=disabled, 1=I-branch (HF), 2=Q-branch (HF)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[ds_range]
            ))

        self.declare_parameter('device_index', 0,
            ParameterDescriptor(
                description='RTL-SDR device index for multi-dongle setups (0-15)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[dev_range]
            ))

        self.declare_parameter('agc_mode', False,
            ParameterDescriptor(description='Enable RTL2832U internal AGC'))

        self.declare_parameter('publish_raw_iq', True,
            ParameterDescriptor(description='Publish raw IQ samples (high bandwidth)'))

        self.declare_parameter('publish_spectrum', True,
            ParameterDescriptor(description='Compute and publish FFT spectrum'))

        self.declare_parameter('fft_size', 1024,
            ParameterDescriptor(
                description='FFT size for spectrum computation (256-8192)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[fft_size_range]
            ))

        self.declare_parameter('spectrum_averaging', 4,
            ParameterDescriptor(
                description='Number of spectra to average',
                type=ParameterType.PARAMETER_INTEGER
            ))

        # Get parameters
        self.auto_connect = self.get_parameter('auto_connect').value
        self.auto_stream = self.get_parameter('auto_stream').value
        self.center_frequency = self.get_parameter('center_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.gain = self.get_parameter('gain').value
        self.bias_tee = self.get_parameter('bias_tee').value
        self.ppm_correction = self.get_parameter('ppm_correction').value
        self.direct_sampling = self.get_parameter('direct_sampling').value
        self.device_index = self.get_parameter('device_index').value
        self.agc_mode = self.get_parameter('agc_mode').value
        self.publish_raw_iq = self.get_parameter('publish_raw_iq').value
        self.publish_spectrum = self.get_parameter('publish_spectrum').value
        self.fft_size = self.get_parameter('fft_size').value
        self.spectrum_averaging = self.get_parameter('spectrum_averaging').value

        # ====================================================================
        # Publishers
        # ====================================================================

        if self.publish_raw_iq:
            self.iq_pub = self.create_publisher(Float32MultiArray, '~/iq_samples', 10)

        if self.publish_spectrum:
            self.spectrum_pub = self.create_publisher(Float32MultiArray, '~/spectrum', 10)

        self.status_pub = self.create_publisher(String, '~/device_status', 10)
        self.signal_strength_pub = self.create_publisher(Float32, '~/signal_strength', 10)

        # ====================================================================
        # Services
        # ====================================================================

        self.start_stream_srv = self.create_service(
            Trigger, '~/start_stream', self.start_stream_callback)
        self.stop_stream_srv = self.create_service(
            Trigger, '~/stop_stream', self.stop_stream_callback)
        self.set_bias_tee_srv = self.create_service(
            SetBool, '~/set_bias_tee', self.set_bias_tee_callback)

        # ====================================================================
        # Internal State
        # ====================================================================

        self.device_connected = False
        self.streaming = False
        self.device_info = {}
        self.stream_process = None
        self.stream_thread = None
        self.stop_streaming_flag = threading.Event()

        # Spectrum averaging buffer
        self.spectrum_buffer = []

        # ====================================================================
        # Initialize
        # ====================================================================

        self.get_logger().info('RTL-SDR V4 Node initialized')
        self.get_logger().info(f'  Center Frequency: {self.center_frequency/1e6:.3f} MHz')
        self.get_logger().info(f'  Sample Rate: {self.sample_rate/1e6:.3f} MSPS')
        self.get_logger().info(f'  Gain: {"auto" if self.gain < 0 else f"{self.gain:.1f} dB"}')
        self.get_logger().info(f'  Bias Tee: {self.bias_tee}')
        self.get_logger().info(f'  PPM Correction: {self.ppm_correction}')
        self.get_logger().info(f'  Direct Sampling: {self.direct_sampling}')
        self.get_logger().info(f'  Device Index: {self.device_index}')

        # Check for rtl-sdr tools
        if not self.check_rtlsdr_tools():
            self.get_logger().error('rtl-sdr tools not found! Please install rtl-sdr.')
            self.get_logger().error('For V4 support: https://github.com/rtlsdrblog/rtl-sdr-blog')
            self.get_logger().error('Or run: scripts/rtl_sdr/install_rtl_sdr.sh')
            return

        # Auto-connect if enabled
        if self.auto_connect:
            if self.detect_device():
                self.device_connected = True
                self.publish_status('Connected')

                # Set bias tee if enabled
                if self.bias_tee:
                    self.apply_bias_tee(True)

                # Auto-stream if enabled
                if self.auto_stream:
                    self.start_streaming()
            else:
                self.get_logger().warn('No RTL-SDR device detected')
                self.publish_status('No device found')

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update_callback)

        # ====================================================================
        # Dynamic Reconfigure: Parameter Change Callback
        # ====================================================================

        self.add_on_set_parameters_callback(self.parameters_callback)

    # ====================================================================
    # Device Detection
    # ====================================================================

    def check_rtlsdr_tools(self) -> bool:
        """Check if rtl-sdr command-line tools are available."""
        try:
            result = subprocess.run(
                ['rtl_test', '-t'],
                capture_output=True,
                timeout=3.0
            )
            # rtl_test -t exits quickly; returncode 0 or device-not-found still
            # means the binary exists
            return True
        except FileNotFoundError:
            return False
        except subprocess.TimeoutExpired:
            # Binary exists but took too long (possibly stuck talking to device)
            return True

    def detect_device(self) -> bool:
        """Detect RTL-SDR device and get basic info."""
        try:
            # rtl_test -t does a quick device probe
            result = subprocess.run(
                ['rtl_test', '-d', str(self.device_index), '-t'],
                capture_output=True,
                text=True,
                timeout=5.0
            )

            output = result.stdout + result.stderr

            if 'Found' in output and 'device' in output.lower():
                self.get_logger().info(f'RTL-SDR Device Found:\n{output.strip()}')
                self.device_info = self.parse_device_info(output)
                return True
            elif 'no supported devices' in output.lower() or 'no device' in output.lower():
                self.get_logger().warn(f'No RTL-SDR device found at index {self.device_index}')
                return False
            else:
                # Some output but unclear -- assume present if binary ran
                self.get_logger().info(f'rtl_test output: {output.strip()}')
                self.device_info = self.parse_device_info(output)
                return True

        except subprocess.TimeoutExpired:
            self.get_logger().error('rtl_test timeout -- device may be in use')
            return False
        except Exception as e:
            self.get_logger().error(f'Error detecting device: {e}')
            return False

    def parse_device_info(self, output: str) -> dict:
        """Parse rtl_test output for device info."""
        info = {}
        for line in output.split('\n'):
            line_lower = line.lower().strip()
            if 'using device' in line_lower:
                # "Using device 0: Generic RTL2832U OEM"
                parts = line.split(':', 1)
                if len(parts) > 1:
                    info['name'] = parts[1].strip()
            elif 'rtl2832u' in line_lower or 'r828d' in line_lower or 'r860' in line_lower:
                info['chipset'] = line.strip()
            elif 'serial' in line_lower:
                parts = line.split(':')
                if len(parts) > 1:
                    info['serial'] = parts[1].strip()
            elif 'tuner type' in line_lower or 'tuner is' in line_lower:
                info['tuner'] = line.strip()
        return info

    # ====================================================================
    # Bias Tee Control
    # ====================================================================

    def apply_bias_tee(self, enable: bool):
        """Enable or disable the RTL-SDR V4 bias tee using rtl_biast."""
        try:
            cmd = ['rtl_biast', '-d', str(self.device_index),
                   '-b', '1' if enable else '0']
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
            if result.returncode == 0:
                self.get_logger().info(f'Bias tee {"enabled" if enable else "disabled"}')
            else:
                self.get_logger().warn(f'rtl_biast failed: {result.stderr}')
        except FileNotFoundError:
            self.get_logger().warn('rtl_biast not found -- bias tee control unavailable')
            self.get_logger().warn('Install rtl-sdr-blog fork for V4 bias tee support')
        except Exception as e:
            self.get_logger().error(f'Bias tee error: {e}')

    # ====================================================================
    # Dynamic Reconfigure
    # ====================================================================

    def parameters_callback(self, parameters):
        """Handle dynamic parameter changes."""
        result = SetParametersResult(successful=True)
        was_streaming = self.streaming

        # Parameters that require stream restart
        needs_restart = False

        for param in parameters:
            param_name = param.name
            param_value = param.value
            param_type = param.type_

            try:
                if param_name == 'center_frequency':
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        freq_mhz = param_value / 1e6
                        if 24.0 <= freq_mhz <= 1766.0:
                            self.center_frequency = param_value
                            self.get_logger().info(
                                f'Center frequency changed to {freq_mhz:.3f} MHz')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = (
                                f'Frequency {freq_mhz:.3f} MHz out of range (24-1766 MHz)')

                elif param_name == 'sample_rate':
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        if 225.001e3 <= param_value <= 3.2e6:
                            self.sample_rate = param_value
                            self.get_logger().info(
                                f'Sample rate changed to {param_value/1e6:.3f} MSPS')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = (
                                f'Sample rate must be 225 kHz - 3.2 MSPS, '
                                f'got {param_value/1e6:.3f}')

                elif param_name == 'gain':
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        if param_value < 0:
                            self.gain = -1.0  # auto
                            self.get_logger().info('Gain set to auto')
                            needs_restart = True
                        elif 0.0 <= param_value <= 49.6:
                            # Snap to nearest valid gain step
                            nearest = min(RTLSDR_GAIN_VALUES,
                                          key=lambda g: abs(g - param_value))
                            self.gain = nearest
                            self.get_logger().info(
                                f'Gain changed to {nearest:.1f} dB '
                                f'(nearest valid step)')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = (
                                f'Gain must be -1 (auto) or 0-49.6 dB, '
                                f'got {param_value}')

                elif param_name == 'bias_tee':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.bias_tee = param_value
                        self.apply_bias_tee(param_value)

                elif param_name == 'ppm_correction':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if -500 <= param_value <= 500:
                            self.ppm_correction = param_value
                            self.get_logger().info(
                                f'PPM correction changed to {param_value}')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'PPM must be -500..500, got {param_value}'

                elif param_name == 'direct_sampling':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if param_value in (0, 1, 2):
                            self.direct_sampling = param_value
                            modes = {0: 'disabled', 1: 'I-branch', 2: 'Q-branch'}
                            self.get_logger().info(
                                f'Direct sampling: {modes[param_value]}')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = (
                                f'Direct sampling must be 0, 1, or 2, '
                                f'got {param_value}')

                elif param_name == 'agc_mode':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.agc_mode = param_value
                        self.get_logger().info(
                            f'AGC {"enabled" if param_value else "disabled"}')
                        needs_restart = True

                elif param_name == 'publish_raw_iq':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.publish_raw_iq = param_value
                        if param_value and not hasattr(self, 'iq_pub'):
                            self.iq_pub = self.create_publisher(
                                Float32MultiArray, '~/iq_samples', 10)
                        self.get_logger().info(f'Publish raw IQ: {param_value}')

                elif param_name == 'publish_spectrum':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.publish_spectrum = param_value
                        if param_value and not hasattr(self, 'spectrum_pub'):
                            self.spectrum_pub = self.create_publisher(
                                Float32MultiArray, '~/spectrum', 10)
                        self.get_logger().info(f'Publish spectrum: {param_value}')

                elif param_name == 'fft_size':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 256 <= param_value <= 8192 and param_value % 256 == 0:
                            self.fft_size = param_value
                            self.spectrum_buffer.clear()
                            self.get_logger().info(f'FFT size changed to {param_value}')
                        else:
                            result.successful = False
                            result.reason = (
                                f'FFT size must be 256-8192 in steps of 256, '
                                f'got {param_value}')

                elif param_name == 'spectrum_averaging':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if param_value > 0:
                            self.spectrum_averaging = param_value
                            self.get_logger().info(
                                f'Spectrum averaging changed to {param_value}')
                        else:
                            result.successful = False
                            result.reason = f'Averaging must be > 0, got {param_value}'

                else:
                    self.get_logger().warn(f'Unknown parameter: {param_name}')

            except Exception as e:
                result.successful = False
                result.reason = f'Error setting {param_name}: {str(e)}'
                self.get_logger().error(f'Parameter callback error: {e}')

        # Restart streaming if needed and was streaming
        if needs_restart and was_streaming and result.successful:
            self.get_logger().info('Restarting stream with new parameters...')
            self.stop_streaming()
            time.sleep(0.5)
            self.start_streaming()

        return result

    # ====================================================================
    # Streaming
    # ====================================================================

    def start_streaming(self):
        """Start RTL-SDR streaming in a background thread."""
        if self.streaming:
            self.get_logger().warn('Already streaming')
            return

        self.get_logger().info('Starting RTL-SDR stream...')
        self.stop_streaming_flag.clear()
        self.streaming = True

        self.stream_thread = threading.Thread(
            target=self._stream_worker, daemon=True)
        self.stream_thread.start()

        self.publish_status('Streaming')

    def stop_streaming(self):
        """Stop RTL-SDR streaming."""
        if not self.streaming:
            return

        self.get_logger().info('Stopping RTL-SDR stream...')
        self.streaming = False
        self.stop_streaming_flag.set()

        if self.stream_process and self.stream_process.poll() is None:
            self.stream_process.terminate()
            try:
                self.stream_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.stream_process.kill()

        if self.stream_thread:
            self.stream_thread.join(timeout=3.0)

        self.publish_status('Connected (idle)')

    def _stream_worker(self):
        """Background worker that runs rtl_sdr and processes IQ data.

        rtl_sdr outputs raw unsigned 8-bit interleaved I/Q to stdout.
        We convert to float32 in [-1, 1] range for downstream processing.
        """
        try:
            # Build rtl_sdr command
            # rtl_sdr pipes 8-bit unsigned IQ to stdout when output file is '-'
            cmd = [
                'rtl_sdr',
                '-d', str(self.device_index),
                '-f', str(int(self.center_frequency)),
                '-s', str(int(self.sample_rate)),
                '-p', str(self.ppm_correction),
            ]

            # Gain: omit -g for auto, otherwise specify
            if self.gain >= 0:
                cmd.extend(['-g', f'{self.gain:.1f}'])
            else:
                # Auto gain -- do not pass -g flag, rtl_sdr defaults to auto
                pass

            # Direct sampling
            if self.direct_sampling > 0:
                cmd.extend(['-D', str(self.direct_sampling)])

            # AGC
            if self.agc_mode:
                cmd.extend(['-A', '1'])

            # Output to stdout ('-')
            cmd.append('-')

            self.get_logger().info(f'Running: {" ".join(cmd)}')

            self.stream_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0  # Unbuffered for low latency
            )

            # Give process time to initialize
            time.sleep(1.0)

            if self.stream_process.poll() is not None:
                stderr_output = self.stream_process.stderr.read().decode(
                    'utf-8', errors='ignore')
                self.get_logger().error(
                    f'rtl_sdr exited immediately with code '
                    f'{self.stream_process.returncode}')
                self.get_logger().error(f'Error: {stderr_output}')
                if 'usb' in stderr_output.lower() or 'device' in stderr_output.lower():
                    self.get_logger().error(
                        'Possible USB or device-in-use issue. '
                        'Try: sudo rmmod dvb_usb_rtl28xxu')
                raise RuntimeError(f'rtl_sdr failed to start: {stderr_output}')

            self.get_logger().info('rtl_sdr streaming, reading IQ from stdout...')

            # RTL-SDR outputs unsigned 8-bit IQ: each sample is 2 bytes (I, Q)
            # Read in chunks for efficiency
            samples_per_chunk = 1024  # IQ sample pairs
            chunk_size = samples_per_chunk * 2  # 2 bytes per IQ pair

            no_data_count = 0
            max_no_data = 100

            while not self.stop_streaming_flag.is_set() and self.streaming:
                # Check process health
                if self.stream_process.poll() is not None:
                    stderr_output = self.stream_process.stderr.read().decode(
                        'utf-8', errors='ignore')
                    self.get_logger().error(
                        f'rtl_sdr died with code '
                        f'{self.stream_process.returncode}')
                    if stderr_output:
                        self.get_logger().error(f'Error: {stderr_output}')
                    break

                # Read raw 8-bit unsigned IQ from stdout
                data = self.stream_process.stdout.read(chunk_size)

                if not data:
                    no_data_count += 1
                    if no_data_count > max_no_data:
                        self.get_logger().warn(
                            'No data from rtl_sdr -- possible hardware issue')
                        break
                    continue

                no_data_count = 0

                if len(data) >= chunk_size:
                    # Convert unsigned 8-bit to float32 in [-1.0, 1.0]
                    raw = np.frombuffer(data, dtype=np.uint8)
                    samples = (raw.astype(np.float32) - 127.5) / 127.5

                    # Separate I and Q (already interleaved: I0,Q0,I1,Q1,...)
                    i_samples = samples[0::2]
                    q_samples = samples[1::2]

                    # Publish raw IQ
                    if self.publish_raw_iq:
                        self.publish_iq_samples(i_samples, q_samples)

                    # Compute and publish spectrum
                    if self.publish_spectrum:
                        self.compute_and_publish_spectrum(i_samples, q_samples)

        except Exception as e:
            self.get_logger().error(f'Streaming error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.streaming = False
            self.publish_status('Connected (idle)')

    # ====================================================================
    # Data Publishing
    # ====================================================================

    def publish_iq_samples(self, i_samples: np.ndarray, q_samples: np.ndarray):
        """Publish raw IQ samples as interleaved float32."""
        msg = Float32MultiArray()

        iq_interleaved = np.empty(
            (i_samples.size + q_samples.size,), dtype=np.float32)
        iq_interleaved[0::2] = i_samples
        iq_interleaved[1::2] = q_samples

        msg.data = iq_interleaved.tolist()

        dim = MultiArrayDimension()
        dim.label = 'samples'
        dim.size = len(iq_interleaved)
        dim.stride = len(iq_interleaved)
        msg.layout.dim.append(dim)

        self.iq_pub.publish(msg)

    def compute_and_publish_spectrum(
        self, i_samples: np.ndarray, q_samples: np.ndarray
    ):
        """Compute FFT spectrum and publish."""
        iq = i_samples + 1j * q_samples

        # Pad or truncate to FFT size
        if len(iq) > self.fft_size:
            iq = iq[:self.fft_size]
        elif len(iq) < self.fft_size:
            iq = np.pad(iq, (0, self.fft_size - len(iq)))

        # Apply Hanning window
        window = np.hanning(self.fft_size)
        iq_windowed = iq * window

        # Compute FFT
        spectrum = np.fft.fftshift(np.fft.fft(iq_windowed))
        power_db = 20 * np.log10(np.abs(spectrum) + 1e-12)

        # Average
        self.spectrum_buffer.append(power_db)
        if len(self.spectrum_buffer) > self.spectrum_averaging:
            self.spectrum_buffer.pop(0)

        averaged_spectrum = np.mean(self.spectrum_buffer, axis=0)

        # Publish spectrum
        msg = Float32MultiArray()
        msg.data = averaged_spectrum.tolist()

        dim = MultiArrayDimension()
        dim.label = 'frequency_bins'
        dim.size = len(averaged_spectrum)
        dim.stride = len(averaged_spectrum)
        msg.layout.dim.append(dim)

        self.spectrum_pub.publish(msg)

        # Publish signal strength
        max_power = float(np.max(averaged_spectrum))
        self.signal_strength_pub.publish(Float32(data=max_power))

    def publish_status(self, status: str):
        """Publish device status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def status_update_callback(self):
        """Periodic status update."""
        if self.streaming:
            if self.stream_process and self.stream_process.poll() is not None:
                self.get_logger().error('Stream process died unexpectedly')
                self.streaming = False
                self.publish_status('Error: stream died')
            else:
                self.publish_status('Streaming')
        elif self.device_connected:
            self.publish_status('Connected (idle)')
        else:
            self.publish_status('Disconnected')

    # ====================================================================
    # Service Callbacks
    # ====================================================================

    def start_stream_callback(self, request, response):
        """Service callback to start streaming."""
        try:
            self.start_streaming()
            response.success = True
            response.message = 'Streaming started'
        except Exception as e:
            response.success = False
            response.message = f'Failed to start: {e}'
        return response

    def stop_stream_callback(self, request, response):
        """Service callback to stop streaming."""
        try:
            self.stop_streaming()
            response.success = True
            response.message = 'Streaming stopped'
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop: {e}'
        return response

    def set_bias_tee_callback(self, request, response):
        """Service callback to enable/disable bias tee."""
        try:
            self.bias_tee = request.data
            self.apply_bias_tee(request.data)
            response.success = True
            response.message = (
                f'Bias tee {"enabled" if self.bias_tee else "disabled"}')
        except Exception as e:
            response.success = False
            response.message = f'Failed to set bias tee: {e}'
        return response

    def __del__(self):
        """Cleanup on destruction."""
        if self.bias_tee:
            self.apply_bias_tee(False)
        self.stop_streaming()


def main(args=None):
    rclpy.init(args=args)
    node = RTLSDRNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_streaming()
        if node.bias_tee:
            node.apply_bias_tee(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
