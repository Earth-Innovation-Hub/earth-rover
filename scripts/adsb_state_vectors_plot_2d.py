#!/usr/bin/env python3
"""
2D plot of ADS-B aircraft states as a published ROS image.

Subscribes to aircraft state vectors (JSON). Renders trike at origin and aircraft
positions in local ENU (East–North) within a configurable range (default 100 km),
with velocity arrows and optional ICAO labels. Publishes the plot as sensor_msgs/Image
(no GUI).

Usage:
  ros2 run deepgis_vehicles adsb_state_vectors_plot_2d.py

  # View with: ros2 run rqt_image_view rqt_image_view
  # Or: rqt -> Plugins -> Visualization -> Image View, topic ~/aircraft_plot_image
"""

import json
import sys
import threading

def _import_matplotlib():
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        return plt
    except (ImportError, AttributeError) as e:
        err = str(e)
        if 'numpy' in err.lower() or 'ARRAY_API' in err or 'multiarray' in err:
            print(
                'Matplotlib/NumPy version conflict. Fix: pip install "numpy<2"\n',
                file=sys.stderr,
            )
        raise

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Header


M_PER_KM = 1000.0


class StateVectorsPlot2DNode(Node):
    def __init__(self):
        super().__init__('adsb_state_vectors_plot_2d')

        self.declare_parameter('state_vectors_topic', '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors')
        self.declare_parameter('range_km', 100.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 800)
        self.declare_parameter('show_velocity_arrows', True)
        self.declare_parameter('velocity_arrow_scale', 80.0)
        self.declare_parameter('show_labels', True)

        self._topic = self.get_parameter('state_vectors_topic').value
        self._range_km = float(self.get_parameter('range_km').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._show_vel = self.get_parameter('show_velocity_arrows').value
        self._vel_scale = float(self.get_parameter('velocity_arrow_scale').value)
        self._show_labels = self.get_parameter('show_labels').value

        self._lock = threading.Lock()
        self._latest = None

        self.create_subscription(String, self._topic, self._callback, 10)
        self._image_pub = self.create_publisher(Image, '~/aircraft_plot_image', 10)

        self._fig = None
        self._ax = None
        self._trike_plot = None
        self._ac_scatter = None
        self._quiver_ref = [None]
        self._label_artists = []
        self._plt = None

        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_plot)

        self.get_logger().info(
            '2D plot image: topic=%s, range=±%.0f km, publish %.1f Hz' % (self._topic, self._range_km, self._rate_hz)
        )

    def _callback(self, msg):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._latest = data

    def _get_latest(self):
        with self._lock:
            return self._latest

    def _setup_figure(self):
        if self._fig is not None:
            return
        self._plt = _import_matplotlib()
        plt = self._plt
        range_m = self._range_km * M_PER_KM
        lim = range_m

        self._fig, self._ax = plt.subplots(figsize=(self._img_w / 100.0, self._img_h / 100.0), dpi=100)
        ax = self._ax
        ax.set_aspect('equal')
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_xlabel('East (m)', fontsize=9)
        ax.set_ylabel('North (m)', fontsize=9)
        ax.set_title('Aircraft (%.0f km), velocity arrows' % (2 * self._range_km), fontsize=10)
        ax.grid(True, alpha=0.5)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.axvline(0, color='k', linewidth=0.5)

        self._trike_plot, = ax.plot([0], [0], 'o', color='red', markersize=10, markeredgecolor='darkred', markeredgewidth=2, label='Trike', zorder=5)
        self._ac_scatter = ax.scatter([], [], c='#00b0ff', s=35, alpha=0.9, edgecolors='#0066aa', linewidths=0.5, label='Aircraft', zorder=4)
        ax.legend(loc='upper right', fontsize=7)
        self._fig.subplots_adjust(left=0.09, right=0.95, top=0.93, bottom=0.09)
        self._fig.patch.set_facecolor('white')
        ax.set_facecolor('white')

    def _update_artists(self):
        data = self._get_latest()
        if not data:
            return

        ax = self._ax
        for a in self._label_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._label_artists.clear()

        if self._quiver_ref[0] is not None:
            self._quiver_ref[0].remove()
            self._quiver_ref[0] = None

        aircraft_list = data.get('aircraft') or []
        xs, ys = [], []
        uvs, vvs = [], []
        icaos = []

        for ac in aircraft_list:
            pose = ac.get('pose') or {}
            pos = pose.get('position') or {}
            x = float(pos.get('x', 0.0))
            y = float(pos.get('y', 0.0))
            xs.append(x)
            ys.append(y)
            if self._show_vel:
                twist = ac.get('twist') or {}
                lin = twist.get('linear') or {}
                u = float(lin.get('x', 0.0)) * self._vel_scale
                v = float(lin.get('y', 0.0)) * self._vel_scale
                uvs.append(u)
                vvs.append(v)
            icaos.append((ac.get('icao') or '').strip())

        self._trike_plot.set_data([0], [0])

        if xs:
            self._ac_scatter.set_offsets([[x, y] for x, y in zip(xs, ys)])
            self._ac_scatter.set_sizes([35] * len(xs))
        else:
            self._ac_scatter.set_offsets([])

        if self._show_vel and xs and len(uvs) == len(xs):
            q = ax.quiver(
                xs, ys, uvs, vvs,
                color='#cc6600', alpha=0.9, scale=1.0, scale_units='xy', angles='xy',
                width=0.004, headwidth=5, headlength=6, zorder=3,
            )
            self._quiver_ref[0] = q

        if self._show_labels and xs and icaos:
            for xi, yi, icao in zip(xs, ys, icaos):
                if icao:
                    t = ax.text(xi, yi, '  ' + icao, fontsize=6, color='#222', alpha=0.9, clip_on=True, zorder=6)
                    self._label_artists.append(t)

    def _publish_plot(self):
        self._setup_figure()
        self._update_artists()

        self._fig.canvas.draw()
        w, h = self._fig.canvas.get_width_height()
        raw = self._fig.canvas.tostring_rgb()
        if len(raw) != w * h * 3:
            return

        msg = Image()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='')
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = list(raw)
        self._image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateVectorsPlot2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
