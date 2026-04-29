#!/usr/bin/env python3
"""
Spectrum Analyzer Node for SDR

Advanced spectrum analysis for HydraSDR or any IQ source.
Provides waterfall display, peak detection, and signal monitoring.

Topics Subscribed:
  ~/iq_samples (std_msgs/Float32MultiArray): Raw IQ samples from SDR

Topics Published:
  ~/spectrum (std_msgs/Float32MultiArray): Power spectrum in dB
  ~/waterfall (sensor_msgs/Image): Waterfall display data
  ~/peaks (std_msgs/Float32MultiArray): Detected spectral peaks
  ~/peak_frequencies (std_msgs/Float32MultiArray): Frequencies of peaks

Parameters:
  fft_size (int): FFT size (default: 2048)
  window_type (string): Window function - hanning, hamming, blackman, rectangular
  averaging (int): Number of spectra to average (default: 4)
  waterfall_depth (int): Waterfall history depth (default: 100)
  peak_threshold_db (float): Peak detection threshold above noise floor (default: 10.0)
  sample_rate (float): Sample rate in Hz for frequency axis (default: 2.5e6)
  center_frequency (float): Center frequency in Hz (default: 433.0e6)
"""

import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image


class SpectrumAnalyzerNode(Node):
    """Advanced spectrum analyzer for SDR signals."""
    
    def __init__(self):
        super().__init__('spectrum_analyzer_node')
        
        # ====================================================================
        # Declare Parameters
        # ====================================================================
        
        self.declare_parameter('fft_size', 2048)
        self.declare_parameter('window_type', 'hanning')
        self.declare_parameter('averaging', 4)
        self.declare_parameter('waterfall_depth', 100)
        self.declare_parameter('peak_threshold_db', 10.0)
        self.declare_parameter('sample_rate', 2.5e6)
        self.declare_parameter('center_frequency', 433.0e6)
        
        # Get parameters
        self.fft_size = self.get_parameter('fft_size').value
        self.window_type = self.get_parameter('window_type').value
        self.averaging = self.get_parameter('averaging').value
        self.waterfall_depth = self.get_parameter('waterfall_depth').value
        self.peak_threshold_db = self.get_parameter('peak_threshold_db').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.center_frequency = self.get_parameter('center_frequency').value
        
        # ====================================================================
        # Create Window Function
        # ====================================================================
        
        if self.window_type == 'hanning':
            self.window = np.hanning(self.fft_size)
        elif self.window_type == 'hamming':
            self.window = np.hamming(self.fft_size)
        elif self.window_type == 'blackman':
            self.window = np.blackman(self.fft_size)
        elif self.window_type == 'rectangular':
            self.window = np.ones(self.fft_size)
        else:
            self.get_logger().warn(f'Unknown window type: {self.window_type}, using Hanning')
            self.window = np.hanning(self.fft_size)
        
        # ====================================================================
        # Subscribers
        # ====================================================================
        
        self.iq_sub = self.create_subscription(
            Float32MultiArray,
            '~/iq_samples',
            self.iq_callback,
            10
        )
        
        # ====================================================================
        # Publishers
        # ====================================================================
        
        self.spectrum_pub = self.create_publisher(Float32MultiArray, '~/spectrum', 10)
        self.waterfall_pub = self.create_publisher(Image, '~/waterfall', 10)
        self.peaks_pub = self.create_publisher(Float32MultiArray, '~/peaks', 10)
        self.peak_freqs_pub = self.create_publisher(Float32MultiArray, '~/peak_frequencies', 10)
        
        # ====================================================================
        # Internal State
        # ====================================================================
        
        self.spectrum_buffer = deque(maxlen=self.averaging)
        self.waterfall_buffer = deque(maxlen=self.waterfall_depth)
        
        # Frequency axis
        self.freq_axis = np.fft.fftshift(np.fft.fftfreq(self.fft_size, 1/self.sample_rate))
        self.freq_axis += self.center_frequency
        
        # Statistics
        self.sample_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info('Spectrum Analyzer Node initialized')
        self.get_logger().info(f'  FFT Size: {self.fft_size}')
        self.get_logger().info(f'  Window: {self.window_type}')
        self.get_logger().info(f'  Averaging: {self.averaging}')
        self.get_logger().info(f'  Sample Rate: {self.sample_rate/1e6:.3f} MSPS')
        self.get_logger().info(f'  Center Freq: {self.center_frequency/1e6:.3f} MHz')
    
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
            
            # Process in chunks of FFT size
            num_chunks = len(iq) // self.fft_size
            
            for i in range(num_chunks):
                start_idx = i * self.fft_size
                end_idx = start_idx + self.fft_size
                iq_chunk = iq[start_idx:end_idx]
                
                # Compute spectrum for this chunk
                self.compute_spectrum(iq_chunk)
            
            self.sample_count += len(iq)
            
            # Log statistics periodically
            now = self.get_clock().now()
            if (now - self.last_log_time).nanoseconds > 5e9:  # Every 5 seconds
                self.get_logger().info(f'Processed {self.sample_count} samples')
                self.last_log_time = now
                
        except Exception as e:
            self.get_logger().error(f'Error in IQ callback: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def compute_spectrum(self, iq: np.ndarray):
        """Compute and publish spectrum from IQ samples."""
        if len(iq) != self.fft_size:
            return
        
        # Apply window
        iq_windowed = iq * self.window
        
        # Compute FFT
        spectrum = np.fft.fftshift(np.fft.fft(iq_windowed))
        
        # Convert to power in dB
        power_db = 20 * np.log10(np.abs(spectrum) + 1e-12)
        
        # Add to averaging buffer
        self.spectrum_buffer.append(power_db)
        
        # Compute averaged spectrum
        if len(self.spectrum_buffer) > 0:
            averaged_spectrum = np.mean(self.spectrum_buffer, axis=0)
            
            # Publish spectrum
            self.publish_spectrum(averaged_spectrum)
            
            # Detect and publish peaks
            self.detect_and_publish_peaks(averaged_spectrum)
            
            # Add to waterfall
            self.waterfall_buffer.append(averaged_spectrum)
            
            # Publish waterfall
            if len(self.waterfall_buffer) >= 10:  # Need some history
                self.publish_waterfall()
    
    def publish_spectrum(self, spectrum: np.ndarray):
        """Publish spectrum data."""
        msg = Float32MultiArray()
        msg.data = spectrum.tolist()
        
        dim = MultiArrayDimension()
        dim.label = 'frequency_bins'
        dim.size = len(spectrum)
        dim.stride = len(spectrum)
        msg.layout.dim.append(dim)
        
        self.spectrum_pub.publish(msg)
    
    def detect_and_publish_peaks(self, spectrum: np.ndarray):
        """Detect spectral peaks above threshold."""
        # Estimate noise floor (median of spectrum)
        noise_floor = np.median(spectrum)
        
        # Find peaks above threshold
        threshold = noise_floor + self.peak_threshold_db
        
        # Simple peak detection: local maxima above threshold
        peaks = []
        peak_freqs = []
        
        for i in range(1, len(spectrum) - 1):
            if (spectrum[i] > threshold and 
                spectrum[i] > spectrum[i-1] and 
                spectrum[i] > spectrum[i+1]):
                peaks.append(float(spectrum[i]))
                peak_freqs.append(float(self.freq_axis[i]))
        
        # Publish peaks
        if peaks:
            peaks_msg = Float32MultiArray()
            peaks_msg.data = peaks
            self.peaks_pub.publish(peaks_msg)
            
            freqs_msg = Float32MultiArray()
            freqs_msg.data = peak_freqs
            self.peak_freqs_pub.publish(freqs_msg)
    
    def publish_waterfall(self):
        """Publish waterfall display as an image."""
        try:
            if len(self.waterfall_buffer) == 0:
                return
            
            # Convert waterfall buffer to 2D array
            waterfall_array = np.array(self.waterfall_buffer)
            
            # Ensure we have valid data
            if waterfall_array.size == 0:
                return
            
            # Ensure 2D array (time x frequency)
            if len(waterfall_array.shape) == 1:
                # Single spectrum, reshape to 1 row
                waterfall_array = waterfall_array.reshape(1, -1)
            
            # Normalize to 0-255 for display
            wf_min = np.min(waterfall_array)
            wf_max = np.max(waterfall_array)
            
            if wf_max > wf_min:
                # Normalize and scale to 0-255
                waterfall_normalized = ((waterfall_array - wf_min) / (wf_max - wf_min) * 255).astype(np.uint8)
            else:
                # All values are the same, use zeros
                waterfall_normalized = np.zeros_like(waterfall_array, dtype=np.uint8)
            
            # Create ROS Image message directly (no cv_bridge needed)
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'spectrum_analyzer'
            img_msg.height = waterfall_normalized.shape[0]
            img_msg.width = waterfall_normalized.shape[1]
            img_msg.encoding = 'mono8'
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width  # For mono8, step = width
            img_msg.data = waterfall_normalized.tobytes()
            
            self.waterfall_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing waterfall: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SpectrumAnalyzerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

