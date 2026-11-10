#!/usr/bin/env python3
"""
DeepGIS Keyframe Publisher

Posts georeferenced camera keyframes to DeepGIS (/label/api/vehicles/frame/), so following
the rover on the globe shows what it sees, and the AI pilot can describe it. A keyframe is
sent when the rover has driven distance_m, turned turn_deg, or heartbeat_sec has passed,
never more often than min_interval_sec and never above max_kbps on average.

The camera is not subscribed between keyframes: for each one a subscription is created, one
raw frame taken and the subscription destroyed, so the 15 Hz stream is not copied all the
time. The raw topic is used because the compressed one JPEG-encodes the Bayer mosaic as
grayscale, which loses the colour.

With spectrometer_topic set, each keyframe is followed by a spectral product
(/label/api/vehicles/product/): the spectrometer reading integrated across the moment the
frame was taken, reduced by spectral_product.py, with the laser range to the target. The
frame is held for up to SPECTRUM_WAIT_SEC for that reading; if none comes it goes alone.

Same bearer key as the pose publisher (key_file, mode 600).
"""

import json
import math
import os
import queue
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64, Float64MultiArray

import spectral_product

FRESH_SEC = 5.0
SPECTRUM_FRESH_SEC = 2.0   # the spectrometer counts as running if it reported this recently
SPECTRUM_WAIT_SEC = 1.5    # how long a frame waits for the reading that spans it
LASER_KEEP = 64            # laser samples kept (10 Hz: about 6 s)
HEADING_FRESH_SEC = 3.0
COURSE_MIN_SPEED = 0.5
GRAB_TIMEOUT_SEC = 3.0
EARTH_R = 6371000.0

BAYER = {
    'bayer_rggb8': cv2.COLOR_BayerBG2BGR,
    'bayer_bggr8': cv2.COLOR_BayerRG2BGR,
    'bayer_gbrg8': cv2.COLOR_BayerGR2BGR,
    'bayer_grbg8': cv2.COLOR_BayerGB2BGR,
}


def to_bgr(msg):
    """sensor_msgs/Image (8-bit) as a BGR array, or None for an unsupported encoding."""
    enc = msg.encoding.lower()
    channels = 3 if enc in ('rgb8', 'bgr8') else 1
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    a = np.ascontiguousarray(a[:, :msg.width * channels]).reshape(msg.height, msg.width, channels)
    if enc in BAYER:
        return cv2.cvtColor(a[..., 0], BAYER[enc])
    if enc == 'rgb8':
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if enc == 'bgr8':
        return a
    if enc == 'mono8':
        return cv2.cvtColor(a[..., 0], cv2.COLOR_GRAY2BGR)
    return None


def gray_world(bgr):
    means = bgr.reshape(-1, 3).mean(axis=0)
    if means.min() < 1.0:
        return bgr
    gains = np.clip(means.mean() / means, 0.5, 3.0)
    return np.clip(bgr * gains, 0, 255).astype(np.uint8)


def distance_m(a, b):
    dlat = math.radians(b[0] - a[0])
    dlon = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
    return EARTH_R * math.hypot(dlat, dlon)


class DeepGISKeyframePublisher(Node):
    def __init__(self):
        super().__init__('deepgis_keyframe_publisher')

        self.declare_parameter('deepgis_url', 'https://deepgis.org')
        self.declare_parameter('key_file', os.path.expanduser('~/.config/earth-rover/deepgis_vehicle.key'))
        self.declare_parameter('vehicle_id', 'santacruz')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('image_topic', '/stereo/left/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('white_balance', True)
        self.declare_parameter('distance_m', 10.0)
        self.declare_parameter('turn_deg', 30.0)
        self.declare_parameter('heartbeat_sec', 30.0)
        self.declare_parameter('min_interval_sec', 5.0)
        self.declare_parameter('max_kbps', 64.0)
        # Camera mount relative to the rover: pitch (negative = down) and horizontal field
        # of view; hfov 0 = unknown (not sent).
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('hfov_deg', 0.0)
        # Spectrometer and laser ranger mounted along this camera's axis; '' = camera only.
        # /spectrometer is [integration_us, N counts, N wavelengths_nm]. fov is the full
        # acceptance cone (25 deg for a bare 0.22 NA fibre, 14 for the bare f/4 port);
        # boresight is where it points in the image, as fractions from the left and top.
        self.declare_parameter('spectrometer_topic', '/spectrometer')
        self.declare_parameter('laser_topic', '/laser_distance')
        self.declare_parameter('spectrometer_fov_deg', 25.0)
        self.declare_parameter('boresight_u', 0.5)
        self.declare_parameter('boresight_v', 0.5)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = p
        self.url = p('deepgis_url').rstrip('/') + '/label/api/vehicles/frame/'
        self.product_url = p('deepgis_url').rstrip('/') + '/label/api/vehicles/product/'
        self.vehicle = p('vehicle_id')
        with open(os.path.expanduser(p('key_file'))) as f:
            key = f.read().strip()
        self.http = requests.Session()
        self.http.headers.update({'Authorization': f'Bearer {key}', 'Content-Type': 'image/jpeg'})

        self.fix = None
        self.fix_t = 0.0
        self.heading = None
        self.heading_t = 0.0
        self.course = None
        self.speed = None

        self.last_pos = None
        self.last_heading = None
        self.last_sent = -math.inf
        self.next_allowed = 0.0
        self.grab = None
        self.grab_t = 0.0
        self.failures = 0
        self.sent = 0
        self.spectrum = None      # (arrival time, message data)
        self.laser = deque(maxlen=LASER_KEEP)
        self.pending = None       # (jpeg, meta, time): a frame waiting for its spectrum
        self.product_failures = 0
        self.products_sent = 0

        if p('spectrometer_topic'):
            self.create_subscription(Float64MultiArray, p('spectrometer_topic'), self.on_spectrum,
                                     qos_profile_sensor_data)
            if p('laser_topic'):
                self.create_subscription(Float64, p('laser_topic'), self.on_laser, qos_profile_sensor_data)
        ns = p('mavros_namespace').rstrip('/')
        self.create_subscription(NavSatFix, f'{ns}/global_position/raw/fix', self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Float64, f'{ns}/global_position/compass_hdg', self.on_heading, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', self.on_velocity,
                                 qos_profile_sensor_data)
        self.outbox = queue.Queue(maxsize=1)
        threading.Thread(target=self.uploader, daemon=True).start()
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'Posting {self.vehicle} keyframes from {p("image_topic")} to {self.url}')

    def on_fix(self, msg):
        if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.fix, self.fix_t = msg, time.monotonic()

    def on_heading(self, msg):
        self.heading, self.heading_t = float(msg.data), time.monotonic()

    def on_velocity(self, msg):
        v = msg.twist.linear
        self.speed = math.hypot(v.x, v.y)
        if self.speed >= COURSE_MIN_SPEED:
            self.course = math.degrees(math.atan2(v.x, v.y)) % 360.0

    def current_heading(self):
        if (self.heading is not None and math.isfinite(self.heading)
                and time.monotonic() - self.heading_t <= HEADING_FRESH_SEC):
            return self.heading
        return self.course

    def current_fix(self):
        if self.fix is None or time.monotonic() - self.fix_t > FRESH_SEC:
            return None
        return self.fix

    def due(self, now):
        if now - self.last_sent < self.p('min_interval_sec') or now < self.next_allowed:
            return None
        if now - self.last_sent >= self.p('heartbeat_sec'):
            return 'heartbeat'
        fix = self.current_fix()
        if fix is not None and self.last_pos is not None:
            if distance_m(self.last_pos, (fix.latitude, fix.longitude)) >= self.p('distance_m'):
                return 'distance'
        h = self.current_heading()
        if h is not None and self.last_heading is not None:
            if abs((h - self.last_heading + 180.0) % 360.0 - 180.0) >= self.p('turn_deg'):
                return 'turn'
        return None

    def on_laser(self, msg):
        self.laser.append((time.monotonic(), float(msg.data)))

    def on_spectrum(self, msg):
        self.spectrum = (time.monotonic(), msg.data)
        if self.pending is not None:
            self.send(self.spectrum)

    def make_product(self, spectrum, frame_t):
        """The spectral product for a reading that arrived at spectrum[0], or None."""
        t, data = spectrum
        n = (len(data) - 1) // 2
        if n < 64:
            return None
        integration_s = float(data[0]) * 1e-6
        start = t - integration_s - 0.1
        ranges = [r for (lt, r) in self.laser if start <= lt <= t]
        product = spectral_product.build(
            np.asarray(data[1 + n:1 + 2 * n]), np.asarray(data[1:1 + n]), data[0], ranges,
            fov_deg=float(self.p('spectrometer_fov_deg')),
            boresight=(float(self.p('boresight_u')), float(self.p('boresight_v'))),
            camera_hfov_deg=float(self.p('hfov_deg')))
        if product is not None:
            # when the reading was integrated, in seconds relative to the camera frame
            product['t_rel'] = [round(t - integration_s - frame_t, 2), round(t - frame_t, 2)]
        return product

    def send(self, spectrum):
        """Queue the pending frame, with the product from this spectrum if there is one."""
        body, meta, frame_t = self.pending
        self.pending = None
        product = None
        if spectrum is not None:
            try:
                product = self.make_product(spectrum, frame_t)
            except Exception as e:  # a bad reading must not cost the frame
                self._product_failed(f'{type(e).__name__}: {e}')
        try:
            self.outbox.put_nowait((body, meta, product))
        except queue.Full:
            pass

    def tick(self):
        now = time.monotonic()
        if self.pending is not None and now - self.pending[2] > SPECTRUM_WAIT_SEC:
            self.send(None)
        if self.grab is not None:
            if now - self.grab_t > GRAB_TIMEOUT_SEC:
                self.destroy_subscription(self.grab)
                self.grab = None
                self.last_sent = now  # no camera: wait a full interval before trying again
                self._failed(f'no image on {self.p("image_topic")} within {GRAB_TIMEOUT_SEC:.0f} s')
            return
        reason = self.due(now)
        if reason is None or self.outbox.full() or self.pending is not None:
            return
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.grab_reason = reason
        self.grab_t = now
        self.grab = self.create_subscription(Image, self.p('image_topic'), self.on_image, qos)

    def on_image(self, msg):
        if self.grab is None:
            return
        self.destroy_subscription(self.grab)
        self.grab = None
        now = time.monotonic()
        self.last_sent = now
        fix = self.current_fix()
        heading = self.current_heading()
        if fix is not None:
            self.last_pos = (fix.latitude, fix.longitude)
        if heading is not None:
            self.last_heading = heading

        bgr = to_bgr(msg)
        if bgr is None:
            self._failed(f'unsupported image encoding {msg.encoding}')
            return
        w = int(self.p('width'))
        if 0 < w < bgr.shape[1]:
            bgr = cv2.resize(bgr, (w, round(bgr.shape[0] * w / bgr.shape[1])), interpolation=cv2.INTER_AREA)
        if self.p('white_balance'):
            bgr = gray_world(bgr)
        ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.p('jpeg_quality'))])
        if not ok:
            self._failed('JPEG encoding failed')
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        meta = {'vehicle': self.vehicle, 'captured': stamp if stamp > 1e9 else time.time(),
                'gimbal_pitch_deg': float(self.p('pitch_deg')), 'trigger': self.grab_reason}
        if self.p('hfov_deg') > 0:
            meta['hfov_deg'] = float(self.p('hfov_deg'))
        if fix is not None:
            meta['lat'], meta['lon'] = fix.latitude, fix.longitude
            if math.isfinite(fix.altitude):
                meta['alt_amsl'] = fix.altitude
        if heading is not None:
            meta['heading_deg'] = heading
        if self.speed is not None and math.isfinite(self.speed):
            meta['speed_mps'] = self.speed
        body = jpg.tobytes()
        self.next_allowed = now + len(body) * 8 / (max(1.0, float(self.p('max_kbps'))) * 1000.0)
        self.pending = (body, meta, now)
        if self.spectrum is None or now - self.spectrum[0] > SPECTRUM_FRESH_SEC:
            self.send(None)  # no spectrometer running: the frame goes alone, now
        # otherwise the next reading is the one integrated across this frame: on_spectrum sends both

    def uploader(self):
        while True:
            body, meta, product = self.outbox.get()
            try:
                r = self.http.post(self.url, data=body, headers={'X-Frame-Meta': json.dumps(meta)}, timeout=15.0)
            except requests.RequestException as e:
                self._failed(f'{type(e).__name__}: {e}')
                continue
            if r.status_code != 200:
                self._failed(f'HTTP {r.status_code}: {r.text[:200]}')
                continue
            self.sent += 1
            if self.failures or self.sent == 1:
                self.get_logger().info(f'DeepGIS accepted keyframe ({len(body)} B, {meta["trigger"]})')
            self.failures = 0
            if product is not None:
                self.post_product(product, meta)

    def post_product(self, product, meta):
        payload = json.dumps({'vehicle': self.vehicle, 'product': 'spectrum', 'captured': meta['captured'],
                              'data': product}, separators=(',', ':'))
        try:
            r = self.http.post(self.product_url, data=payload, headers={'Content-Type': 'application/json'},
                               timeout=15.0)
        except requests.RequestException as e:
            self._product_failed(f'{type(e).__name__}: {e}')
            return
        if r.status_code != 200:
            self._product_failed(f'HTTP {r.status_code}: {r.text[:200]}')
            return
        self.products_sent += 1
        if self.product_failures or self.products_sent == 1:
            self.get_logger().info(f'DeepGIS accepted spectral product ({len(payload)} B)')
        self.product_failures = 0

    def _product_failed(self, why):
        self.product_failures += 1
        if self.product_failures in (1, 10) or self.product_failures % 60 == 0:
            self.get_logger().warn(f'Spectral product not sent ({self.product_failures}x): {why}')

    def _failed(self, why):
        self.failures += 1
        if self.failures in (1, 10) or self.failures % 60 == 0:
            self.get_logger().warn(f'Keyframe not sent ({self.failures}x): {why}')


def main(args=None):
    rclpy.init(args=args)
    node = DeepGISKeyframePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
