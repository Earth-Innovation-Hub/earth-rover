#!/usr/bin/env python3
"""
Efficient Real-Time SDR Visualizer

High-performance visualization for HydraSDR spectrum and waterfall data.
Uses pyqtgraph for efficient real-time plotting.

Features:
- Real-time spectrum plot with peak markers
- Waterfall display with configurable history
- IQ constellation plot (optional)
- Efficient data handling with downsampling
- Configurable update rates

Usage:
    ros2 run deepgis_vehicles sdr_visualizer.py
"""

import sys
import numpy as np
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

try:
    import pyqtgraph as pg
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QLabel,
        QPushButton, QDoubleSpinBox, QSpinBox, QCheckBox, QGroupBox, QGridLayout
    )
    from PyQt5.QtCore import Qt, QTimer
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False
    print("Warning: PyQt5 and pyqtgraph required for visualization")
    print("Install with: pip install pyqt5 pyqtgraph")


class SDRVisualizerNode(Node):
    """Efficient ROS2 node for visualizing SDR data."""
    
    def __init__(self):
        super().__init__('sdr_visualizer')
        
        # Parameters
        self.declare_parameter('spectrum_topic', '/hydra_sdr/hydra_sdr_node/spectrum')
        self.declare_parameter('waterfall_topic', '/hydra_sdr/spectrum_analyzer_node/waterfall')
        self.declare_parameter('iq_topic', '/hydra_sdr/hydra_sdr_node/iq_samples')
        self.declare_parameter('center_frequency', 433.0e6)
        self.declare_parameter('sample_rate', 2.5e6)
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.declare_parameter('waterfall_depth', 200)
        self.declare_parameter('show_constellation', False)
        self.declare_parameter('downsample_factor', 1)  # Downsample for performance
        self.declare_parameter('sdr_node_name', '/hydra_sdr/hydra_sdr_node')  # Node to control
        self.declare_parameter('show_controls', True)  # Show parameter control panel
        
        # Get parameters
        spectrum_topic = self.get_parameter('spectrum_topic').value
        waterfall_topic = self.get_parameter('waterfall_topic').value
        iq_topic = self.get_parameter('iq_topic').value
        self.center_freq = self.get_parameter('center_frequency').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        self.waterfall_depth = self.get_parameter('waterfall_depth').value
        self.show_constellation = self.get_parameter('show_constellation').value
        self.downsample = self.get_parameter('downsample_factor').value
        self.sdr_node_name = self.get_parameter('sdr_node_name').value
        self.show_controls = self.get_parameter('show_controls').value
        
        # Data buffers
        self.spectrum_data = None
        self.waterfall_buffer = deque(maxlen=self.waterfall_depth)
        self.iq_buffer = deque(maxlen=1000)  # For constellation
        
        # Frequency axis
        self.freq_axis = None
        
        # Statistics
        self.spectrum_count = 0
        self.waterfall_count = 0
        self.iq_count = 0
        
        # Subscribers
        self.spectrum_sub = self.create_subscription(
            Float32MultiArray,
            spectrum_topic,
            self.spectrum_callback,
            10
        )
        
        self.waterfall_sub = self.create_subscription(
            Image,
            waterfall_topic,
            self.waterfall_callback,
            10
        )
        
        if self.show_constellation:
            self.iq_sub = self.create_subscription(
                Float32MultiArray,
                iq_topic,
                self.iq_callback,
                10
            )
        
        self.get_logger().info('SDR Visualizer Node initialized')
        self.get_logger().info(f'  Spectrum topic: {spectrum_topic}')
        self.get_logger().info(f'  Waterfall topic: {waterfall_topic}')
        self.get_logger().info(f'  Center frequency: {self.center_freq/1e6:.3f} MHz')
        self.get_logger().info(f'  Sample rate: {self.sample_rate/1e6:.3f} MSPS')
        
        # Initialize GUI if available
        if PYQT_AVAILABLE:
            self.init_gui()
        else:
            self.get_logger().warn('GUI not available - running in headless mode')
    
    def init_gui(self):
        """Initialize PyQt GUI."""
        self.app = QApplication(sys.argv) if not QApplication.instance() else QApplication.instance()
        
        # Main window
        self.window = QMainWindow()
        self.window.setWindowTitle('HydraSDR Visualizer')
        self.window.resize(1200, 800)
        
        # Central widget
        central_widget = QWidget()
        self.window.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Status bar
        status_layout = QHBoxLayout()
        self.status_label = QLabel('Waiting for data...')
        self.status_label.setStyleSheet('font-weight: bold; padding: 5px;')
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Parameter control panel
        if self.show_controls:
            control_group = QGroupBox('SDR Parameters (Dynamic Reconfigure)')
            control_layout = QGridLayout()
            control_group.setLayout(control_layout)
            
            # Frequency control
            control_layout.addWidget(QLabel('Frequency (MHz):'), 0, 0)
            self.freq_spinbox = QDoubleSpinBox()
            self.freq_spinbox.setRange(24.0, 1800.0)
            self.freq_spinbox.setValue(self.center_freq / 1e6)
            self.freq_spinbox.setDecimals(3)
            self.freq_spinbox.setSingleStep(1.0)
            control_layout.addWidget(self.freq_spinbox, 0, 1)
            
            # Sample rate control
            control_layout.addWidget(QLabel('Sample Rate (MSPS):'), 0, 2)
            self.sample_rate_spinbox = QDoubleSpinBox()
            self.sample_rate_spinbox.setRange(2.5, 10.0)
            self.sample_rate_spinbox.setValue(self.sample_rate / 1e6)
            self.sample_rate_spinbox.setDecimals(1)
            self.sample_rate_spinbox.setSingleStep(2.5)
            control_layout.addWidget(self.sample_rate_spinbox, 0, 3)
            
            # Gain control
            control_layout.addWidget(QLabel('Gain (dB):'), 1, 0)
            self.gain_spinbox = QSpinBox()
            self.gain_spinbox.setRange(0, 21)
            self.gain_spinbox.setValue(20)
            control_layout.addWidget(self.gain_spinbox, 1, 1)
            
            # Bias tee control
            self.bias_tee_checkbox = QCheckBox('Bias Tee')
            self.bias_tee_checkbox.setChecked(False)
            control_layout.addWidget(self.bias_tee_checkbox, 1, 2)
            
            # Apply button
            apply_btn = QPushButton('Apply Parameters')
            apply_btn.clicked.connect(self.apply_parameters)
            control_layout.addWidget(apply_btn, 1, 3)
            
            layout.addWidget(control_group)
        
        # Spectrum plot
        spectrum_widget = pg.PlotWidget(title='Power Spectrum (dB)')
        spectrum_widget.setLabel('left', 'Power', units='dB')
        spectrum_widget.setLabel('bottom', 'Frequency', units='MHz')
        spectrum_widget.showGrid(x=True, y=True, alpha=0.3)
        self.spectrum_plot = spectrum_widget.plot([], [], pen='y', name='Spectrum')
        self.peak_markers = []
        layout.addWidget(spectrum_widget, stretch=2)
        
        # Waterfall plot
        waterfall_widget = pg.PlotWidget(title='Waterfall')
        waterfall_widget.setLabel('left', 'Time')
        waterfall_widget.setLabel('bottom', 'Frequency', units='MHz')
        waterfall_widget.setAspectLocked(False)
        self.waterfall_img = pg.ImageItem()
        waterfall_widget.addItem(self.waterfall_img)
        # Set up colormap
        colormap = pg.colormap.get('viridis')
        self.waterfall_img.setLookupTable(colormap.getLookupTable())
        layout.addWidget(waterfall_widget, stretch=2)
        
        # Constellation plot (optional)
        if self.show_constellation:
            constellation_widget = pg.PlotWidget(title='IQ Constellation')
            constellation_widget.setLabel('left', 'Q')
            constellation_widget.setLabel('bottom', 'I')
            constellation_widget.showGrid(x=True, y=True, alpha=0.3)
            self.constellation_plot = constellation_widget.plot([], [], pen=None, symbol='o', symbolSize=2)
            layout.addWidget(constellation_widget, stretch=1)
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(int(1000 / self.update_rate))
        
        self.window.show()
    
    def spectrum_callback(self, msg):
        """Handle spectrum data."""
        try:
            spectrum = np.array(msg.data, dtype=np.float32)
            
            # Downsample if needed
            if self.downsample > 1 and len(spectrum) > self.downsample:
                spectrum = spectrum[::self.downsample]
            
            self.spectrum_data = spectrum
            
            # Update frequency axis if needed
            if self.freq_axis is None or len(self.freq_axis) != len(spectrum):
                n = len(spectrum)
                # Frequency bins relative to center
                freq_offset = np.fft.fftshift(np.fft.fftfreq(n, 1/self.sample_rate))
                self.freq_axis = (self.center_freq + freq_offset) / 1e6  # Convert to MHz
            
            self.spectrum_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing spectrum: {e}')
    
    def waterfall_callback(self, msg):
        """Handle waterfall image data."""
        try:
            # Convert image data to numpy array
            if msg.encoding == 'mono8':
                height = msg.height
                width = msg.width
                data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
                self.waterfall_buffer.append(data)
                self.waterfall_count += 1
            else:
                self.get_logger().warn(f'Unsupported waterfall encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Error processing waterfall: {e}')
    
    def iq_callback(self, msg):
        """Handle IQ samples for constellation."""
        try:
            iq_data = np.array(msg.data, dtype=np.float32)
            if len(iq_data) >= 2:
                # Separate I and Q
                i_samples = iq_data[0::2]
                q_samples = iq_data[1::2]
                # Downsample for performance
                step = max(1, len(i_samples) // 1000)
                self.iq_buffer.extend(zip(i_samples[::step], q_samples[::step]))
                self.iq_count += 1
        except Exception as e:
            self.get_logger().error(f'Error processing IQ: {e}')
    
    def update_gui(self):
        """Update GUI elements."""
        if not PYQT_AVAILABLE:
            return
        
            # Update spectrum plot
        if self.spectrum_data is not None and self.freq_axis is not None:
            # Efficient downsampling for display (max 2000 points for smooth rendering)
            if len(self.spectrum_data) > 2000:
                step = len(self.spectrum_data) // 2000
                spectrum_display = self.spectrum_data[::step]
                freq_display = self.freq_axis[::step]
            else:
                spectrum_display = self.spectrum_data
                freq_display = self.freq_axis
            
            # Use efficient setData (pyqtgraph handles this efficiently)
            self.spectrum_plot.setData(freq_display, spectrum_display)
            
            # Find and mark peaks
            if len(spectrum_display) > 10:
                # Simple peak detection
                threshold = np.percentile(spectrum_display, 95)
                peaks = []
                for i in range(1, len(spectrum_display) - 1):
                    if (spectrum_display[i] > threshold and
                        spectrum_display[i] > spectrum_display[i-1] and
                        spectrum_display[i] > spectrum_display[i+1]):
                        peaks.append((freq_display[i], spectrum_display[i]))
                
                # Clear old markers
                for marker in self.peak_markers:
                    self.spectrum_plot.scene().removeItem(marker)
                self.peak_markers.clear()
                
                # Add new markers for top 5 peaks
                peaks.sort(key=lambda x: x[1], reverse=True)
                for freq, power in peaks[:5]:
                    marker = pg.InfiniteLine(pos=freq, angle=90, pen='r', movable=False)
                    self.spectrum_plot.addItem(marker)
                    self.peak_markers.append(marker)
        
        # Update waterfall (only if new data)
        if len(self.waterfall_buffer) > 0:
            # Convert deque to array efficiently
            waterfall_array = np.array(list(self.waterfall_buffer))
            # Transpose for display (time on y-axis, frequency on x-axis)
            # Use autoLevels=False and manual levels for better performance
            if waterfall_array.size > 0:
                self.waterfall_img.setImage(waterfall_array, autoLevels=False)
        
        # Update constellation (downsampled for performance)
        if self.show_constellation and len(self.iq_buffer) > 0:
            # Limit to last 5000 points for performance
            iq_list = list(self.iq_buffer)[-5000:]
            iq_array = np.array(iq_list)
            i_samples = iq_array[:, 0]
            q_samples = iq_array[:, 1]
            self.constellation_plot.setData(i_samples, q_samples)
        
        # Update status
        status_text = (
            f'Spectrum: {self.spectrum_count} | '
            f'Waterfall: {self.waterfall_count} | '
            f'IQ: {self.iq_count}'
        )
        self.status_label.setText(status_text)
    
    def apply_parameters(self):
        """Apply parameter changes to SDR node using ROS2 parameter service."""
        try:
            # Use command-line interface to set parameters
            import subprocess
            
            node_name = self.sdr_node_name.replace('/', '')
            if node_name.startswith('/'):
                node_name = node_name[1:]
            
            params_to_set = [
                ('center_frequency', str(self.freq_spinbox.value() * 1e6)),
                ('sample_rate', str(self.sample_rate_spinbox.value() * 1e6)),
                ('gain', str(self.gain_spinbox.value())),
                ('bias_tee', 'true' if self.bias_tee_checkbox.isChecked() else 'false'),
            ]
            
            for param_name, param_value in params_to_set:
                cmd = ['ros2', 'param', 'set', self.sdr_node_name, param_name, param_value]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                if result.returncode == 0:
                    self.get_logger().info(f'Set {param_name} = {param_value}')
                else:
                    self.get_logger().warn(f'Failed to set {param_name}: {result.stderr}')
            
            # Update local values
            self.center_freq = self.freq_spinbox.value() * 1e6
            self.sample_rate = self.sample_rate_spinbox.value() * 1e6
            # Reset frequency axis
            self.freq_axis = None
            self.get_logger().info('Parameters applied successfully')
        
        except Exception as e:
            self.get_logger().error(f'Error applying parameters: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def run(self):
        """Run the visualizer."""
        if PYQT_AVAILABLE:
            # Run ROS2 in a separate thread or use QTimer
            import threading
            
            def spin_ros():
                rclpy.spin(self)
            
            ros_thread = threading.Thread(target=spin_ros, daemon=True)
            ros_thread.start()
            
            # Run Qt event loop
            sys.exit(self.app.exec_())
        else:
            # Headless mode
            rclpy.spin(self)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = SDRVisualizerNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

