#!/usr/bin/env python3
"""
Improved Spectrometer Data Plotting Script

Subscribes to spectrometer topic and publishes real-time intensity vs wavelength
as a ROS Image topic (no GUI by default). Optional plot saving.

Usage:
    ros2 run spectrometery_ros2 Intensity_Plot.py --ros-args ...
    Use --dpi, --fig-width, --fig-height, --max-plot-points, --update-rate to reduce CPU.
"""

import matplotlib
matplotlib.use('Agg')  # No GUI; image publishing only
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import argparse
import os
from datetime import datetime
from collections import deque


class SpectrometerPlotter(Node):
    """ROS 2 node for real-time spectrometer data visualization."""

    def __init__(self, topic_name='spectrometer', save_dir=None, max_history=100, update_rate=10,
                 publish_image=True, image_topic='spectrometer_plot',
                 fig_width=12.0, fig_height=7.0, figure_dpi=100.0, max_plot_points=0,
                 show_dips=True, dip_prominence_frac=0.02, dip_min_distance_samples=0):
        super().__init__('spectrometer_plotter')
        
        self.topic_name = topic_name
        self.save_dir = save_dir
        self.max_history = max_history
        self.update_rate = update_rate
        self.publish_image = publish_image
        self.max_plot_points = max(0, int(max_plot_points))
        self.show_dips = show_dips
        self.dip_prominence_frac = max(1e-9, float(dip_prominence_frac))
        self.dip_min_distance_samples = int(dip_min_distance_samples)
        self._dip_artists = []
        
        # Data storage
        self.wavelengths = None
        self.intensities = None
        self.integration_time = None
        self.data_lock = False  # Simple flag to prevent race conditions
        self.plot_count = 0
        
        # History for statistics
        self.intensity_history = deque(maxlen=max_history)
        self.peak_wavelength_history = deque(maxlen=max_history)
        
        # Create save directory if specified
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f'Plots will be saved to: {self.save_dir}')
        
        # Subscribe to spectrometer topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            self.topic_name,
            self.listener_callback,
            10)
        
        self.get_logger().info(f'Subscribed to topic: {self.topic_name}')
        
        # Publisher for spectral plot image (default: image topic only, no GUI)
        if self.publish_image:
            self.image_publisher = self.create_publisher(Image, image_topic, 10)
            self.get_logger().info(f'Publishing spectral plot images to: {image_topic}')
        
        # Setup matplotlib (Agg backend: no window, render to buffer for publishing)
        self.fig, self.ax = plt.subplots(
            figsize=(float(fig_width), float(fig_height)),
            dpi=float(figure_dpi))
        self.line, = self.ax.plot([], [], 'b-', linewidth=1.5, label='Intensity')
        self.ax.set_xlabel('Wavelength (nm)', fontsize=12)
        self.ax.set_ylabel('Intensity (counts)', fontsize=12)
        self.ax.set_title('Spectrometer Real-Time Plot', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend()
        
        # Text for statistics
        self.stats_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                       verticalalignment='top', fontsize=10,
                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                                       family='monospace')
        
        # Timer for periodic plot updates (throttle to avoid excessive redraws)
        self.update_timer = self.create_timer(1.0 / update_rate, self.update_plot)
        
        mpp = self.max_plot_points
        mpp_msg = f'{mpp} points' if mpp > 0 else 'full resolution'
        dip_msg = f'dip detection on (prom>={self.dip_prominence_frac:.1%} of max I)' if self.show_dips else 'dips off'
        self.get_logger().info(
            f'Spectrometer plotter init: canvas {fig_width}x{fig_height} in @ {figure_dpi} dpi, '
            f'plot series {mpp_msg}, {dip_msg} (image topic only, no GUI).')

    @staticmethod
    def _find_dip_wavelengths(wavelengths, intensities, prominence_frac, min_distance_samples):
        """
        Find absorption-like dips (local intensity minima) using valley prominence.

        prominence_frac: minimum (shoulder height - valley depth) as a fraction of spectrum max.
        min_distance_samples: minimum index spacing between reported dips (0 = auto ~ n/150).
        """
        ii = np.asarray(intensities, dtype=np.float64)
        wl = np.asarray(wavelengths, dtype=np.float64)
        n = ii.size
        if n < 3:
            return np.array([], dtype=np.float64)

        mx = float(np.max(ii))
        if mx <= 0:
            return np.array([], dtype=np.float64)
        prom_abs = max(mx * 1e-6, mx * prominence_frac)

        if min_distance_samples < 1:
            min_distance_samples = max(3, n // 150)

        left = ii[:-2]
        mid = ii[1:-1]
        right = ii[2:]
        is_min = (mid < left) & (mid < right)
        prom = np.minimum(left, right) - mid
        prom[~is_min] = 0.0

        candidates = np.nonzero(prom >= prom_abs)[0] + 1  # shift back to ii indices
        if candidates.size == 0:
            return np.array([], dtype=np.float64)

        order = np.argsort(-prom[candidates - 1])
        candidates = candidates[order]

        picked = []
        for idx in candidates:
            ii_i = int(idx)
            if not picked:
                picked.append(ii_i)
                continue
            if min(abs(ii_i - p) for p in picked) >= min_distance_samples:
                picked.append(ii_i)

        idxs = np.array(sorted(picked), dtype=np.int64)
        return wl[idxs]

    @staticmethod
    def _series_for_plot(wl, ii, max_points):
        """Return (wl, ii) possibly downsampled for drawing only; stats use full arrays."""
        if max_points < 1 or len(wl) <= max_points:
            return wl, ii
        idx = np.linspace(0, len(wl) - 1, num=max_points, dtype=np.int64)
        idx = np.unique(idx)
        return wl[idx], ii[idx]

    def listener_callback(self, msg):
        """Process incoming spectrometer data."""
        try:
            if self.data_lock:
                return  # Skip if currently updating plot
            
            if len(msg.data) < 3:
                self.get_logger().warn(f'Received message with insufficient data: {len(msg.data)} elements')
                return
            
            # Parse data: [integration_time, intensities..., wavelengths...]
            integration_time = msg.data[0]
            half_length = (len(msg.data) - 1) // 2
            
            if half_length < 1:
                self.get_logger().warn('Invalid data structure: half_length < 1')
                return
            
            intensities = np.array(msg.data[1:half_length + 1], dtype=np.float64)
            wavelengths = np.array(msg.data[half_length + 1:], dtype=np.float64)
            
            # Validate data
            if len(intensities) != len(wavelengths):
                self.get_logger().warn(
                    f'Mismatched array lengths: intensities={len(intensities)}, wavelengths={len(wavelengths)}')
                return
            
            if len(intensities) == 0:
                self.get_logger().warn('Received empty intensity array')
                return
            
            # Check for invalid values
            if np.any(np.isnan(intensities)) or np.any(np.isnan(wavelengths)):
                self.get_logger().warn('Received NaN values in data')
                return
            
            if np.any(np.isinf(intensities)) or np.any(np.isinf(wavelengths)):
                self.get_logger().warn('Received Inf values in data')
                return
            
            # Store data
            self.data_lock = True
            self.wavelengths = wavelengths
            self.intensities = intensities
            self.integration_time = integration_time
            
            # Update history for statistics
            max_idx = np.argmax(intensities)
            peak_wavelength = wavelengths[max_idx]
            max_intensity = intensities[max_idx]
            
            self.intensity_history.append(max_intensity)
            self.peak_wavelength_history.append(peak_wavelength)
            
            self.data_lock = False
            
        except Exception as e:
            self.get_logger().error(f'Error processing spectrometer data: {e}')
            self.data_lock = False

    def update_plot(self):
        """Update the plot with latest data."""
        if self.wavelengths is None or self.intensities is None:
            return
        
        try:
            # Plot line uses downsampled data when max_plot_points > 0 (full data still in self.*)
            wl_plot, ii_plot = self._series_for_plot(
                self.wavelengths, self.intensities, self.max_plot_points)
            self.line.set_data(wl_plot, ii_plot)
            
            # Fixed limits from data (cheaper than relim/autoscale_view each frame)
            wl = self.wavelengths
            ii = self.intensities
            y_hi = float(np.max(ii))
            if y_hi <= 0.0:
                y_hi = 1.0
            x_min, x_max = float(np.min(wl)), float(np.max(wl))
            y_top = y_hi * 1.05
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(0.0, y_top)

            dip_wl = (
                self._find_dip_wavelengths(
                    wl, ii, self.dip_prominence_frac, self.dip_min_distance_samples)
                if self.show_dips else np.array([], dtype=np.float64))

            # Dip markers: vertical lines at detected absorption minima
            for dv in self._dip_artists:
                try:
                    dv.remove()
                except (ValueError, NotImplementedError):
                    pass
            self._dip_artists = []
            if dip_wl.size > 0:
                for x in dip_wl:
                    if x_min <= float(x) <= x_max:
                        v = self.ax.axvline(
                            float(x), color='crimson', linestyle='--', linewidth=1.1,
                            alpha=0.75, zorder=3)
                        self._dip_artists.append(v)
            
            # Update title with integration time
            title = f'Spectrometer Real-Time Plot (Integration: {self.integration_time/1000:.1f} ms)'
            self.ax.set_title(title, fontsize=14, fontweight='bold')
            
            # Calculate and display statistics
            max_idx = np.argmax(self.intensities)
            peak_wavelength = self.wavelengths[max_idx]
            max_intensity = self.intensities[max_idx]
            mean_intensity = np.mean(self.intensities)
            std_intensity = np.std(self.intensities)
            
            # History statistics
            if len(self.intensity_history) > 1:
                mean_max_intensity = np.mean(list(self.intensity_history))
                std_max_intensity = np.std(list(self.intensity_history))
                mean_peak_wl = np.mean(list(self.peak_wavelength_history))
            else:
                mean_max_intensity = max_intensity
                std_max_intensity = 0.0
                mean_peak_wl = peak_wavelength
            
            if not self.show_dips:
                dip_line = '\nDips: (disabled)\n'
            elif dip_wl.size > 0:
                dip_preview = ', '.join(f'{x:.1f}' for x in dip_wl[:8])
                if dip_wl.size > 8:
                    dip_preview += ', …'
                dip_line = f'\nDips ({dip_wl.size}): {dip_preview} nm\n'
            else:
                dip_line = '\nDips: none\n'

            stats_str = (
                f'Current:\n'
                f'  Peak λ: {peak_wavelength:.2f} nm\n'
                f'  Max I: {max_intensity:.1f} counts\n'
                f'  Mean I: {mean_intensity:.1f} counts\n'
                f'  Std I: {std_intensity:.1f} counts\n'
                f'{dip_line}'
                f'\nHistory ({len(self.intensity_history)} samples):\n'
                f'  Mean Peak λ: {mean_peak_wl:.2f} nm\n'
                f'  Mean Max I: {mean_max_intensity:.1f} ± {std_max_intensity:.1f}'
            )
            self.stats_text.set_text(stats_str)
            
            # Redraw (Agg: no flush_events needed)
            self.fig.canvas.draw()
            
            # Publish plot as ROS image
            if self.publish_image:
                self.publish_plot_image()
            
            # Optional: save plot periodically
            if self.save_dir and self.plot_count % (self.update_rate * 5) == 0:  # Save every 5 seconds
                self.save_plot()
            
            self.plot_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error updating plot: {e}')

    def publish_plot_image(self):
        """Convert matplotlib figure to ROS Image message and publish."""
        try:
            # Figure must already be drawn (update_plot calls draw once per frame).
            # Avoid a second full Agg render here — it was doubling CPU use.

            # Get the figure as a numpy array
            # Use buffer_rgba() for RGBA format, then convert to RGB
            buf = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
            width, height = self.fig.canvas.get_width_height()
            buf = buf.reshape((height, width, 4))  # RGBA
            
            # Convert RGBA to RGB (drop alpha channel)
            image_array = buf[:, :, :3]  # Take only RGB channels
            
            # Create ROS Image message
            img_msg = Image()
            img_msg.header = Header()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'spectrometer_plot'
            img_msg.height = height
            img_msg.width = width
            img_msg.encoding = 'rgb8'  # RGB8 encoding (standard ROS 2)
            img_msg.is_bigendian = False
            img_msg.step = width * 3  # 3 bytes per pixel (RGB)
            img_msg.data = image_array.tobytes()
            
            # Publish
            self.image_publisher.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing plot image: {e}')

    def save_plot(self):
        """Save current plot to file."""
        if not self.save_dir or self.wavelengths is None:
            return
        
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.save_dir, f'spectrum_{timestamp}.png')
            self.fig.savefig(filename, dpi=150, bbox_inches='tight')
            self.get_logger().info(f'Saved plot: {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving plot: {e}')

    def destroy_node(self):
        """Cleanup on node destruction."""
        if self.save_dir and self.wavelengths is not None:
            self.save_plot()  # Save final plot
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Real-time spectrometer data plotting',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default topic and settings
  ros2 run spectrometer_ocean_optics intensity_plot
  
  # Custom topic
  ros2 run spectrometer_ocean_optics intensity_plot --topic /custom/spectrometer
  
  # Save plots to directory
  ros2 run spectrometer_ocean_optics intensity_plot --save-dir ~/spectra
  
  # Less CPU: lower refresh, smaller canvas, fewer line points
  ros2 run spectrometery_ros2 Intensity_Plot.py --update-rate 5 --dpi 72 --fig-width 8 --fig-height 4 --max-plot-points 1024
        """
    )
    parser.add_argument(
        '--topic', '-t',
        default='spectrometer',
        help='ROS topic name for spectrometer data (default: spectrometer)'
    )
    parser.add_argument(
        '--save-dir', '-s',
        default=None,
        help='Directory to save plots (default: None, no saving)'
    )
    parser.add_argument(
        '--update-rate', '-r',
        type=float,
        default=10.0,
        help='Plot update rate in Hz (default: 10.0)'
    )
    parser.add_argument(
        '--max-history', '-m',
        type=int,
        default=100,
        help='Maximum number of samples for history statistics (default: 100)'
    )
    parser.add_argument(
        '--image-topic', '-i',
        default='spectrometer_plot',
        help='ROS topic name for publishing plot images (default: spectrometer_plot)'
    )
    parser.add_argument(
        '--no-image-pub',
        action='store_true',
        help='Disable publishing plot as ROS image'
    )
    parser.add_argument(
        '--dpi', '-d',
        type=float,
        default=100.0,
        help='Matplotlib figure DPI for rasterized canvas (default: 100). Lower (e.g. 72) reduces CPU.'
    )
    parser.add_argument(
        '--fig-width',
        type=float,
        default=12.0,
        help='Figure width in inches (default: 12)'
    )
    parser.add_argument(
        '--fig-height',
        type=float,
        default=7.0,
        help='Figure height in inches (default: 7)'
    )
    parser.add_argument(
        '--max-plot-points',
        type=int,
        default=0,
        help='Max points drawn for the spectrum line (0 = all points). Downsamples for display only.'
    )
    parser.add_argument(
        '--no-dips',
        action='store_true',
        help='Disable dip detection and vertical markers'
    )
    parser.add_argument(
        '--dip-prominence',
        type=float,
        default=0.02,
        help='Min dip prominence as a fraction of spectrum max intensity E.g. 0.02 = 2%% (default: 0.02)'
    )
    parser.add_argument(
        '--dip-min-distance',
        type=int,
        default=0,
        help='Min index spacing between dips (0 = auto ~ N/150)'
    )
    
    # Parse ROS args and custom args
    ros_args = rclpy.utilities.remove_ros_args(args) if args else []
    parsed_args, unknown = parser.parse_known_args(ros_args)
    
    rclpy.init(args=args)
    
    try:
        plotter = SpectrometerPlotter(
            topic_name=parsed_args.topic,
            save_dir=parsed_args.save_dir,
            max_history=parsed_args.max_history,
            update_rate=parsed_args.update_rate,
            publish_image=not parsed_args.no_image_pub,
            image_topic=parsed_args.image_topic,
            fig_width=parsed_args.fig_width,
            fig_height=parsed_args.fig_height,
            figure_dpi=parsed_args.dpi,
            max_plot_points=parsed_args.max_plot_points,
            show_dips=not parsed_args.no_dips,
            dip_prominence_frac=parsed_args.dip_prominence,
            dip_min_distance_samples=parsed_args.dip_min_distance,
        )
        
        rclpy.spin(plotter)
        
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'plotter' in locals():
            plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
