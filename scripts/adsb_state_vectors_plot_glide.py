#!/usr/bin/env python3
"""
Glide profile plot of ADS-B aircraft: altitude vs glide angle with speed arrows.

Subscribes to aircraft state vectors (JSON). Renders a 2D profile:
  - X-axis: Glide/path angle γ (deg), negative = descent, positive = climb
  - Y-axis: Altitude (ft)
  - Arrows: From each dot toward center (γ=0); length ∝ speed (capped)
  - Scatter: Aircraft positions at (γ, alt); dots and arrows share the same tail
  - Color: Green=climb, Red=descent, Blue=cruise (±50 ft/min)
  - Trike shown as horizontal line at its altitude

Usage:
  ros2 run deepgis_vehicles adsb_state_vectors_plot_glide.py
  ros2 launch deepgis_vehicles adsb_state_vectors_plot_glide.launch.py
"""

import json
import math
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


# ft/min per knot (approx) for glide angle: v_vertical / v_horizontal
# 1 kt ≈ 101.27 ft/min horizontal equivalent for angle computation
FTMIN_PER_KT = 101.27
CRUISE_THRESHOLD_FPM = 50.0  # ±50 ft/min = cruise


class StateVectorsPlotGlideNode(Node):
    def __init__(self):
        super().__init__('adsb_state_vectors_plot_glide')

        self.declare_parameter('state_vectors_topic', '/adsb/adsb_aircraft_state_vectors_node/aircraft_state_vectors')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 800)
        self.declare_parameter('glide_angle_range_deg', 8.0)  # ±range on X
        self.declare_parameter('speed_arrow_scale', 0.008)   # arrow length per kt (capped at ~2.5°)
        self.declare_parameter('altitude_min_ft', 0.0)
        self.declare_parameter('altitude_max_ft', 45000.0)
        self.declare_parameter('show_labels', True)

        self._topic = self.get_parameter('state_vectors_topic').value
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._img_w = int(self.get_parameter('image_width').value)
        self._img_h = int(self.get_parameter('image_height').value)
        self._glide_range = float(self.get_parameter('glide_angle_range_deg').value)
        self._speed_scale = float(self.get_parameter('speed_arrow_scale').value)
        self._alt_min = float(self.get_parameter('altitude_min_ft').value)
        self._alt_max = float(self.get_parameter('altitude_max_ft').value)
        self._show_labels = self.get_parameter('show_labels').value

        self._lock = threading.Lock()
        self._latest = None

        self.create_subscription(String, self._topic, self._callback, 10)
        self._image_pub = self.create_publisher(Image, '~/aircraft_plot_glide_image', 10)

        self._fig = None
        self._ax = None
        self._trike_line = None
        self._fill_descent = None
        self._fill_climb = None
        self._label_artists = []
        self._plt = None

        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_plot)

        self.get_logger().info(
            'Glide profile plot: topic=%s, γ=±%.1f°, alt=%.0f–%.0f ft, %.1f Hz' % (
                self._topic, self._glide_range, self._alt_min, self._alt_max, self._rate_hz)
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

    def _glide_angle_deg(self, speed_kts, vertical_rate_fpm):
        """Glide/path angle γ in degrees. Positive = climb, negative = descent."""
        if speed_kts is None or speed_kts <= 0:
            return 0.0
        v_horiz_fpm = speed_kts * FTMIN_PER_KT
        if v_horiz_fpm <= 0:
            return 0.0
        return math.degrees(math.atan2(float(vertical_rate_fpm or 0), v_horiz_fpm))

    def _flight_mode_color(self, vertical_rate_fpm):
        """Green=climb, Red=descent, Blue=cruise."""
        v = float(vertical_rate_fpm or 0)
        if v > CRUISE_THRESHOLD_FPM:
            return '#2e7d32'   # green - climb
        if v < -CRUISE_THRESHOLD_FPM:
            return '#c62828'   # red - descent
        return '#1565c0'       # blue - cruise

    def _setup_figure(self):
        if self._fig is not None:
            return
        self._plt = _import_matplotlib()
        plt = self._plt

        self._fig, self._ax = plt.subplots(figsize=(self._img_w / 100.0, self._img_h / 100.0), dpi=100)
        ax = self._ax
        ax.set_xlim(-self._glide_range, self._glide_range)
        ax.set_ylim(self._alt_min, self._alt_max)
        ax.set_xlabel('Glide angle γ (°); + = climb', fontsize=9)
        ax.set_ylabel('Altitude (ft)', fontsize=9)
        ax.set_title('Glide profile: alt vs γ; arrows to center = speed', fontsize=10)
        ax.grid(True, alpha=0.5)
        ax.axvline(0, color='k', linewidth=0.5)
        self._fill_descent = ax.fill_between([-self._glide_range, 0], self._alt_min, self._alt_max, alpha=0.05, color='red', label='Descent')
        self._fill_climb = ax.fill_between([0, self._glide_range], self._alt_min, self._alt_max, alpha=0.05, color='green', label='Climb')
        self._trike_line = ax.axhline(0, color='red', linewidth=2, linestyle='--', alpha=0.8, label='Trike alt')
        ax.legend(loc='upper right', fontsize=7)
        self._fig.subplots_adjust(left=0.1, right=0.95, top=0.93, bottom=0.1)
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

        # Clear previous scatter and quiver (preserve fill_between backgrounds)
        for c in list(ax.collections):
            if c not in (self._fill_descent, self._fill_climb):
                c.remove()
        for q in list(ax.containers):
            q.remove()

        aircraft_list = data.get('aircraft') or []
        origin = data.get('header') or {}
        origin_lla = origin.get('origin_lla') or {}
        alt0_m = origin_lla.get('alt_m')
        trike_alt_ft = (float(alt0_m) * 3.28084) if alt0_m is not None else 0.0
        self._trike_line.set_ydata([trike_alt_ft, trike_alt_ft])

        if not aircraft_list:
            self._fig.canvas.draw_idle()
            return

        gammas = []
        alts = []
        colors = []
        icaos = []
        arrow_x_start = []
        arrow_y_start = []
        arrow_u = []
        arrow_v = []

        max_arrow_deg = min(2.5, self._glide_range * 0.4)  # shorten: max ~2.5° length

        for ac in aircraft_list:
            raw = ac.get('raw') or {}
            speed_kts = raw.get('speed_kts')
            if speed_kts is None:
                speed_kts = 0.0
            else:
                speed_kts = float(speed_kts)
            vrate_fpm = raw.get('vertical_rate_ftmin')
            if vrate_fpm is None:
                vrate_fpm = 0.0
            else:
                vrate_fpm = float(vrate_fpm)
            alt_ft = raw.get('altitude_ft')
            if alt_ft is None:
                pos = (ac.get('pose') or {}).get('position') or {}
                up_m = float(pos.get('z', 0.0))
                alt0 = origin_lla.get('alt_m')
                alt_ft = ((up_m + float(alt0)) * 3.28084) if alt0 is not None else (up_m * 3.28084)
            else:
                alt_ft = float(alt_ft)

            gamma = self._glide_angle_deg(speed_kts, vrate_fpm)
            gamma = max(-self._glide_range, min(self._glide_range, gamma))

            gammas.append(gamma)
            alts.append(alt_ft)
            colors.append(self._flight_mode_color(vrate_fpm))
            icaos.append((ac.get('icao') or '').strip())

            # Arrow FROM the dot TOWARD the center (γ=0); length ∝ speed, capped
            # Keeps dot and arrow connected; arrow points to level-flight reference
            arrow_len_deg = min(speed_kts * self._speed_scale, max_arrow_deg)
            u_val = (-arrow_len_deg if gamma > 0 else (arrow_len_deg if gamma < 0 else 0.0))
            arrow_x_start.append(gamma)
            arrow_y_start.append(alt_ft)
            arrow_u.append(u_val)
            arrow_v.append(0.0)

        # Scatter: aircraft positions at (gamma, alt)
        ax.scatter(
            gammas, alts, c=colors, s=40, alpha=0.9, edgecolors='#333', linewidths=0.5, zorder=5
        )
        # Arrows: from dot toward center (shortened; dot and arrow connected)
        ax.quiver(
            arrow_x_start, arrow_y_start, arrow_u, arrow_v,
            color=colors, alpha=0.85, scale=1.0, scale_units='xy', angles='xy',
            width=0.004, headwidth=5, headlength=6, zorder=4
        )

        if self._show_labels:
            for g, a, icao in zip(gammas, alts, icaos):
                if icao:
                    t = ax.text(g, a, '  ' + icao, fontsize=6, color='#222', alpha=0.9, clip_on=True, zorder=6)
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
    node = StateVectorsPlotGlideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
