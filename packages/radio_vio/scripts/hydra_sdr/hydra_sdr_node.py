#!/usr/bin/env python3
"""
HydraSDR ROS2 Node

Wraps the HydraSDR command-line tools (from hydrasdr-host) to provide
ROS2 integration for the HydraSDR RFOne software-defined radio.

This node:
- Detects and manages HydraSDR devices
- Streams IQ samples using hydrasdr_rx or hydrasdr_async_rx
- Publishes raw IQ data, spectrum FFT, and device status
- Provides services for configuration changes

Topics Published:
  ~/iq_samples (std_msgs/Float32MultiArray or sensor_msgs/Image): Raw IQ samples
  ~/spectrum (std_msgs/Float32MultiArray): FFT spectrum
  ~/device_status (std_msgs/String): Device connection and health status
  ~/signal_strength (std_msgs/Float32): Received signal strength indicator

Parameters:
  auto_connect (bool): Automatically connect to first available device
  auto_stream (bool): Automatically start streaming on startup
  center_frequency (float): RF center frequency in Hz
  sample_rate (float): Sample rate in Hz (2.5e6, 5e6, or 10e6)
  gain (int): Receiver gain 0-45 dB
  lna_gain (int): LNA gain 0-15 dB
  mixer_gain (int): Mixer gain 0-15 dB  
  vga_gain (int): VGA gain 0-15 dB
  bias_tee (bool): Enable bias tee for LNA power
  sample_type (int): 0=Float32IQ, 1=Float32Real, 2=Int16IQ, 3=Int16Real, 5=Raw
  publish_raw_iq (bool): Publish raw IQ samples
  publish_spectrum (bool): Compute and publish FFT spectrum
  fft_size (int): FFT size for spectrum computation
  spectrum_averaging (int): Number of spectra to average
  use_soapy (bool): Use SoapySDR instead of direct hydrasdr tools

Dependencies:
  - hydrasdr-host tools installed (hydrasdr_info, hydrasdr_rx)
  - Optional: SoapySDR + SoapyHydraSDR plugin
"""

import os
import sys
import subprocess
import threading
import time
import struct
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
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger


class HydraSDRNode(Node):
    """ROS2 node for HydraSDR RFOne software-defined radio."""
    
    def __init__(self):
        super().__init__('hydra_sdr_node')
        
        # ====================================================================
        # Declare Parameters with Descriptors for Dynamic Reconfigure
        # ====================================================================
        
        # Frequency range: 24-1800 MHz (HydraSDR RFOne spec)
        freq_range = FloatingPointRange()
        freq_range.from_value = 24.0e6
        freq_range.to_value = 1800.0e6
        freq_range.step = 1.0e6
        
        # Sample rate: 2.5, 5, or 10 MSPS
        sample_rate_range = FloatingPointRange()
        sample_rate_range.from_value = 2.5e6
        sample_rate_range.to_value = 10.0e6
        sample_rate_range.step = 2.5e6
        
        # Gain ranges
        gain_range = IntegerRange()
        gain_range.from_value = 0
        gain_range.to_value = 21
        
        lna_gain_range = IntegerRange()
        lna_gain_range.from_value = 0
        lna_gain_range.to_value = 15
        
        mixer_gain_range = IntegerRange()
        mixer_gain_range.from_value = 0
        mixer_gain_range.to_value = 15
        
        vga_gain_range = IntegerRange()
        vga_gain_range.from_value = 0
        vga_gain_range.to_value = 15
        
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
                description='RF center frequency in Hz (24-1800 MHz)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[freq_range]
            ))
        
        self.declare_parameter('sample_rate', 2.5e6,
            ParameterDescriptor(
                description='Sample rate in Hz (2.5, 5.0, or 10.0 MSPS)',
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[sample_rate_range]
            ))
        
        self.declare_parameter('gain', 20,
            ParameterDescriptor(
                description='Linearity gain (0-21 dB)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[gain_range]
            ))
        
        self.declare_parameter('lna_gain', 8,
            ParameterDescriptor(
                description='LNA gain (0-15 dB)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[lna_gain_range]
            ))
        
        self.declare_parameter('mixer_gain', 8,
            ParameterDescriptor(
                description='Mixer gain (0-15 dB)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[mixer_gain_range]
            ))
        
        self.declare_parameter('vga_gain', 8,
            ParameterDescriptor(
                description='VGA gain (0-15 dB)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[vga_gain_range]
            ))
        
        self.declare_parameter('bias_tee', False,
            ParameterDescriptor(description='Enable 4.5V bias tee for LNA/active antenna'))
        
        self.declare_parameter('sample_type', 2,
            ParameterDescriptor(
                description='Sample type: 0=Float32IQ, 1=Float32Real, 2=Int16IQ, 3=Int16Real, 5=Raw',
                type=ParameterType.PARAMETER_INTEGER
            ))
        
        self.declare_parameter('publish_raw_iq', True,
            ParameterDescriptor(description='Publish raw IQ samples (high bandwidth)'))
        
        self.declare_parameter('publish_spectrum', True,
            ParameterDescriptor(description='Compute and publish FFT spectrum'))
        
        self.declare_parameter('fft_size', 1024,
            ParameterDescriptor(
                description='FFT size for spectrum computation (256-8192, step 256)',
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[fft_size_range]
            ))
        
        self.declare_parameter('spectrum_averaging', 4,
            ParameterDescriptor(
                description='Number of spectra to average',
                type=ParameterType.PARAMETER_INTEGER
            ))
        
        self.declare_parameter('use_soapy', False,
            ParameterDescriptor(description='Use SoapySDR backend instead of direct tools'))
        
        self.declare_parameter('serial_number', '',
            ParameterDescriptor(description='Device serial number (empty for auto-detect)'))
        
        # Get parameters
        self.auto_connect = self.get_parameter('auto_connect').value
        self.auto_stream = self.get_parameter('auto_stream').value
        self.center_frequency = self.get_parameter('center_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.gain = self.get_parameter('gain').value
        self.lna_gain = self.get_parameter('lna_gain').value
        self.mixer_gain = self.get_parameter('mixer_gain').value
        self.vga_gain = self.get_parameter('vga_gain').value
        self.bias_tee = self.get_parameter('bias_tee').value
        self.sample_type = self.get_parameter('sample_type').value
        self.publish_raw_iq = self.get_parameter('publish_raw_iq').value
        self.publish_spectrum = self.get_parameter('publish_spectrum').value
        self.fft_size = self.get_parameter('fft_size').value
        self.spectrum_averaging = self.get_parameter('spectrum_averaging').value
        self.use_soapy = self.get_parameter('use_soapy').value
        self.serial_number = self.get_parameter('serial_number').value
        
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
        
        self.get_logger().info('HydraSDR Node initialized')
        self.get_logger().info(f'  Center Frequency: {self.center_frequency/1e6:.3f} MHz')
        self.get_logger().info(f'  Sample Rate: {self.sample_rate/1e6:.3f} MSPS')
        self.get_logger().info(f'  Gain: {self.gain} dB')
        self.get_logger().info(f'  Bias Tee: {self.bias_tee}')
        
        # Check for HydraSDR tools
        if not self.check_hydrasdr_tools():
            self.get_logger().error('HydraSDR tools not found! Please install hydrasdr-host.')
            self.get_logger().error('See: https://github.com/hydrasdr/hydrasdr-host')
            return
        
        # Auto-connect if enabled
        if self.auto_connect:
            if self.detect_device():
                self.device_connected = True
                self.publish_status('Connected')
                
                # Auto-stream if enabled
                if self.auto_stream:
                    self.start_streaming()
            else:
                self.get_logger().warn('No HydraSDR device detected')
                self.publish_status('No device found')
        
        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update_callback)
        
        # ====================================================================
        # Dynamic Reconfigure: Parameter Change Callback
        # ====================================================================
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Track if we need to restart streaming due to parameter changes
        self.need_stream_restart = False
        self.param_change_lock = threading.Lock()
    
    def check_hydrasdr_tools(self) -> bool:
        """Check if HydraSDR command-line tools are available."""
        try:
            result = subprocess.run(['hydrasdr_info'], 
                                    capture_output=True, 
                                    timeout=2.0)
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
    
    def detect_device(self) -> bool:
        """Detect HydraSDR device and get info."""
        try:
            cmd = ['hydrasdr_info']
            if self.serial_number:
                cmd.extend(['-s', self.serial_number])
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                output = result.stdout
                self.get_logger().info(f'HydraSDR Device Found:\n{output}')
                
                # Parse device info
                self.device_info = self.parse_device_info(output)
                return True
            else:
                self.get_logger().warn(f'hydrasdr_info failed: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('hydrasdr_info timeout')
            return False
        except Exception as e:
            self.get_logger().error(f'Error detecting device: {e}')
            return False
    
    def parse_device_info(self, output: str) -> dict:
        """Parse hydrasdr_info output."""
        info = {}
        for line in output.split('\n'):
            if 'Serial Number' in line:
                parts = line.split(':')
                if len(parts) > 1:
                    info['serial'] = parts[1].strip()
            elif 'Board ID' in line:
                parts = line.split(':')
                if len(parts) > 1:
                    info['board_id'] = parts[1].strip()
        return info
    
    def parameters_callback(self, parameters):
        """Handle dynamic parameter changes."""
        result = SetParametersResult(successful=True)
        was_streaming = self.streaming
        
        # Parameters that require stream restart
        stream_restart_params = [
            'center_frequency', 'sample_rate', 'gain', 'lna_gain',
            'mixer_gain', 'vga_gain', 'bias_tee', 'sample_type'
        ]
        
        # Parameters that don't require restart
        runtime_params = [
            'publish_raw_iq', 'publish_spectrum', 'fft_size', 'spectrum_averaging'
        ]
        
        needs_restart = False
        
        for param in parameters:
            param_name = param.name
            param_value = param.value
            param_type = param.type_
            
            try:
                # Validate and apply parameter
                if param_name == 'center_frequency':
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        freq_mhz = param_value / 1e6
                        if 24.0 <= freq_mhz <= 1800.0:
                            self.center_frequency = param_value
                            self.get_logger().info(f'Center frequency changed to {freq_mhz:.3f} MHz')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'Frequency {freq_mhz:.3f} MHz out of range (24-1800 MHz)'
                
                elif param_name == 'sample_rate':
                    if param_type == ParameterType.PARAMETER_DOUBLE:
                        # Validate sample rate (must be 2.5, 5.0, or 10.0 MSPS)
                        valid_rates = [2.5e6, 5.0e6, 10.0e6]
                        if param_value in valid_rates:
                            self.sample_rate = param_value
                            self.get_logger().info(f'Sample rate changed to {param_value/1e6:.1f} MSPS')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'Sample rate must be 2.5, 5.0, or 10.0 MSPS, got {param_value/1e6:.1f}'
                
                elif param_name == 'gain':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 0 <= param_value <= 21:
                            self.gain = param_value
                            self.get_logger().info(f'Gain changed to {param_value} dB')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'Gain must be 0-21, got {param_value}'
                
                elif param_name == 'lna_gain':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 0 <= param_value <= 15:
                            self.lna_gain = param_value
                            self.get_logger().info(f'LNA gain changed to {param_value} dB')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'LNA gain must be 0-15, got {param_value}'
                
                elif param_name == 'mixer_gain':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 0 <= param_value <= 15:
                            self.mixer_gain = param_value
                            self.get_logger().info(f'Mixer gain changed to {param_value} dB')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'Mixer gain must be 0-15, got {param_value}'
                
                elif param_name == 'vga_gain':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 0 <= param_value <= 15:
                            self.vga_gain = param_value
                            self.get_logger().info(f'VGA gain changed to {param_value} dB')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'VGA gain must be 0-15, got {param_value}'
                
                elif param_name == 'bias_tee':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.bias_tee = param_value
                        self.get_logger().info(f'Bias tee {"enabled" if param_value else "disabled"}')
                        needs_restart = True
                
                elif param_name == 'sample_type':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        valid_types = [0, 1, 2, 3, 5]
                        if param_value in valid_types:
                            self.sample_type = param_value
                            self.get_logger().info(f'Sample type changed to {param_value}')
                            needs_restart = True
                        else:
                            result.successful = False
                            result.reason = f'Invalid sample type {param_value}, must be 0,1,2,3, or 5'
                
                elif param_name == 'publish_raw_iq':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.publish_raw_iq = param_value
                        if param_value and not hasattr(self, 'iq_pub'):
                            self.iq_pub = self.create_publisher(Float32MultiArray, '~/iq_samples', 10)
                        self.get_logger().info(f'Publish raw IQ: {param_value}')
                
                elif param_name == 'publish_spectrum':
                    if param_type == ParameterType.PARAMETER_BOOL:
                        self.publish_spectrum = param_value
                        if param_value and not hasattr(self, 'spectrum_pub'):
                            self.spectrum_pub = self.create_publisher(Float32MultiArray, '~/spectrum', 10)
                        self.get_logger().info(f'Publish spectrum: {param_value}')
                
                elif param_name == 'fft_size':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if 256 <= param_value <= 8192 and param_value % 256 == 0:
                            self.fft_size = param_value
                            self.spectrum_buffer.clear()  # Clear buffer for new FFT size
                            self.get_logger().info(f'FFT size changed to {param_value}')
                        else:
                            result.successful = False
                            result.reason = f'FFT size must be 256-8192 in steps of 256, got {param_value}'
                
                elif param_name == 'spectrum_averaging':
                    if param_type == ParameterType.PARAMETER_INTEGER:
                        if param_value > 0:
                            self.spectrum_averaging = param_value
                            self.get_logger().info(f'Spectrum averaging changed to {param_value}')
                        else:
                            result.successful = False
                            result.reason = f'Averaging must be > 0, got {param_value}'
                
                else:
                    # Unknown parameter, allow it but log
                    self.get_logger().warn(f'Unknown parameter: {param_name}')
            
            except Exception as e:
                result.successful = False
                result.reason = f'Error setting {param_name}: {str(e)}'
                self.get_logger().error(f'Parameter callback error: {e}')
        
        # Restart streaming if needed and was streaming
        if needs_restart and was_streaming and result.successful:
            self.get_logger().info('Restarting stream with new parameters...')
            self.stop_streaming()
            time.sleep(0.5)  # Brief pause
            self.start_streaming()
        
        return result
    
    def start_streaming(self):
        """Start SDR streaming in a background thread."""
        if self.streaming:
            self.get_logger().warn('Already streaming')
            return
        
        self.get_logger().info('Starting HydraSDR stream...')
        self.stop_streaming_flag.clear()
        self.streaming = True
        
        # Start streaming thread
        self.stream_thread = threading.Thread(target=self._stream_worker, daemon=True)
        self.stream_thread.start()
        
        self.publish_status('Streaming')
    
    def stop_streaming(self):
        """Stop SDR streaming."""
        if not self.streaming:
            return
        
        self.get_logger().info('Stopping HydraSDR stream...')
        self.streaming = False
        self.stop_streaming_flag.set()
        
        # Terminate process if running
        if self.stream_process and self.stream_process.poll() is None:
            self.stream_process.terminate()
            try:
                self.stream_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.stream_process.kill()
        
        # Wait for thread
        if self.stream_thread:
            self.stream_thread.join(timeout=3.0)
        
        self.publish_status('Connected (idle)')
    
    def _stream_worker(self):
        """Background worker that runs hydrasdr_rx and processes data."""
        try:
            # Use a FIFO for streaming data
            import tempfile
            temp_dir = tempfile.gettempdir()
            fifo_path = os.path.join(temp_dir, f'hydrasdr_fifo_{os.getpid()}')
            
            # Remove old FIFO if exists
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
            
            # Create named pipe
            os.mkfifo(fifo_path)
            
            # Build hydrasdr_rx command
            cmd = [
                'hydrasdr_rx',
                '-r', fifo_path,
                '-f', str(int(self.center_frequency / 1e6)),  # MHz
                '-a', str(int(self.sample_rate)),
                '-t', str(self.sample_type),
                '-g', str(self.gain),
                '-b', '1' if self.bias_tee else '0',
            ]
            
            if self.serial_number:
                cmd.extend(['-s', self.serial_number])
            
            self.get_logger().info(f'Running: {" ".join(cmd)}')
            
            # Start hydrasdr_rx process
            self.stream_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Give process time to start
            time.sleep(1.0)
            
            # Check if process is still running
            if self.stream_process.poll() is not None:
                # Process exited immediately - likely an error
                stderr_output = self.stream_process.stderr.read().decode('utf-8', errors='ignore')
                self.get_logger().error(f'hydrasdr_rx process exited immediately with code {self.stream_process.returncode}')
                self.get_logger().error(f'Error output: {stderr_output}')
                if 'usb' in stderr_output.lower() or 'device' in stderr_output.lower():
                    self.get_logger().error('Possible USB connection or power issue detected!')
                raise RuntimeError(f'hydrasdr_rx failed to start: {stderr_output}')
            
            # Open FIFO for reading
            with open(fifo_path, 'rb') as fifo:
                self.get_logger().info('FIFO opened, streaming data...')
                
                # Determine bytes per sample
                if self.sample_type == 0:  # Float32 IQ
                    bps = 8  # 4 bytes I + 4 bytes Q
                    dtype = np.float32
                elif self.sample_type == 2:  # Int16 IQ
                    bps = 4  # 2 bytes I + 2 bytes Q
                    dtype = np.int16
                else:
                    bps = 4
                    dtype = np.int16
                
                samples_per_chunk = 1024
                chunk_size = samples_per_chunk * bps
                
                # Track data reception for diagnostics
                no_data_count = 0
                max_no_data = 100  # Timeout after 100 empty reads
                
                while not self.stop_streaming_flag.is_set() and self.streaming:
                    # Check if process is still running
                    if self.stream_process.poll() is not None:
                        stderr_output = self.stream_process.stderr.read().decode('utf-8', errors='ignore')
                        self.get_logger().error(f'hydrasdr_rx process died with code {self.stream_process.returncode}')
                        if stderr_output:
                            self.get_logger().error(f'Error output: {stderr_output}')
                        break
                    
                    # Read chunk (blocking read - FIFO will block until data is available)
                    # Note: If hydrasdr_rx isn't producing data, this will block indefinitely
                    # The process check above handles the case where hydrasdr_rx dies
                    data = fifo.read(chunk_size)
                    
                    if not data:
                        no_data_count += 1
                        if no_data_count > max_no_data:
                            self.get_logger().warn('No data received from hydrasdr_rx - possible hardware issue')
                            break
                        continue
                    
                    # Reset no_data_count on successful read
                    no_data_count = 0
                    
                    # Parse IQ samples
                    if len(data) >= chunk_size:
                        if dtype == np.float32:
                            samples = np.frombuffer(data, dtype=np.float32)
                        else:
                            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32)
                        
                        # Separate I and Q
                        i_samples = samples[0::2]
                        q_samples = samples[1::2]
                        
                        # Publish raw IQ
                        if self.publish_raw_iq:
                            self.publish_iq_samples(i_samples, q_samples)
                        
                        # Compute and publish spectrum
                        if self.publish_spectrum:
                            self.compute_and_publish_spectrum(i_samples, q_samples)
            
            # Cleanup
            if os.path.exists(fifo_path):
                os.remove(fifo_path)
                
        except Exception as e:
            self.get_logger().error(f'Streaming error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.streaming = False
            self.publish_status('Connected (idle)')
    
    def publish_iq_samples(self, i_samples: np.ndarray, q_samples: np.ndarray):
        """Publish raw IQ samples."""
        msg = Float32MultiArray()
        
        # Interleave I and Q
        iq_interleaved = np.empty((i_samples.size + q_samples.size,), dtype=np.float32)
        iq_interleaved[0::2] = i_samples
        iq_interleaved[1::2] = q_samples
        
        msg.data = iq_interleaved.tolist()
        
        # Add dimension metadata
        dim = MultiArrayDimension()
        dim.label = 'samples'
        dim.size = len(iq_interleaved)
        dim.stride = len(iq_interleaved)
        msg.layout.dim.append(dim)
        
        self.iq_pub.publish(msg)
    
    def compute_and_publish_spectrum(self, i_samples: np.ndarray, q_samples: np.ndarray):
        """Compute FFT spectrum and publish."""
        # Complex IQ
        iq = i_samples + 1j * q_samples
        
        # Pad or truncate to FFT size
        if len(iq) > self.fft_size:
            iq = iq[:self.fft_size]
        elif len(iq) < self.fft_size:
            iq = np.pad(iq, (0, self.fft_size - len(iq)))
        
        # Apply window
        window = np.hanning(self.fft_size)
        iq_windowed = iq * window
        
        # Compute FFT
        spectrum = np.fft.fftshift(np.fft.fft(iq_windowed))
        power_db = 20 * np.log10(np.abs(spectrum) + 1e-12)
        
        # Average spectra
        self.spectrum_buffer.append(power_db)
        if len(self.spectrum_buffer) > self.spectrum_averaging:
            self.spectrum_buffer.pop(0)
        
        averaged_spectrum = np.mean(self.spectrum_buffer, axis=0)
        
        # Publish
        msg = Float32MultiArray()
        msg.data = averaged_spectrum.tolist()
        
        dim = MultiArrayDimension()
        dim.label = 'frequency_bins'
        dim.size = len(averaged_spectrum)
        dim.stride = len(averaged_spectrum)
        msg.layout.dim.append(dim)
        
        self.spectrum_pub.publish(msg)
        
        # Publish signal strength (max of spectrum)
        max_power = float(np.max(averaged_spectrum))
        self.signal_strength_pub.publish(Float32(data=max_power))
    
    def publish_status(self, status: str):
        """Publish device status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def status_update_callback(self):
        """Periodic status update."""
        # Publish status periodically
        if self.streaming:
            # Check if process is still running
            if self.stream_process and self.stream_process.poll() is not None:
                self.get_logger().error('Stream process died unexpectedly')
                self.streaming = False
                self.publish_status('Error: stream died')
            else:
                # Publish streaming status periodically
                self.publish_status('Streaming')
        elif self.device_connected:
            # Publish idle status periodically
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
            response.success = True
            response.message = f'Bias tee {"enabled" if self.bias_tee else "disabled"}'
            
            # If streaming, need to restart
            if self.streaming:
                self.get_logger().info('Restarting stream with new bias tee setting...')
                self.stop_streaming()
                time.sleep(0.5)
                self.start_streaming()
        except Exception as e:
            response.success = False
            response.message = f'Failed to set bias tee: {e}'
        return response
    
    def __del__(self):
        """Cleanup on destruction."""
        self.stop_streaming()


def main(args=None):
    rclpy.init(args=args)
    node = HydraSDRNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_streaming()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

