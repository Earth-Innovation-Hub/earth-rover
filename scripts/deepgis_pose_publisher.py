#!/usr/bin/env python3
"""
DeepGIS Pose Publisher

Posts this vehicle's live position to DeepGIS (/label/api/vehicles/pose/) so it shows on
the globe, in the Robots list and to the AI pilot. The first accepted pose registers the
vehicle as a field robot; after that each pose updates its last position and track.

The bearer key is DeepGIS's enrollment key for this vehicle id, read from key_file
(mode 600) so it never appears in parameters or the process list. Get it on the DeepGIS
host with `python manage.py vehicle_enroll_key <vehicle_id>`.
"""

import math
import os
import time

import requests
import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

FRESH_SEC = 5.0  # a GPS fix older than this is not posted


class DeepGISPosePublisher(Node):
    def __init__(self):
        super().__init__('deepgis_pose_publisher')

        self.declare_parameter('deepgis_url', 'https://deepgis.org')
        self.declare_parameter('key_file', os.path.expanduser('~/.config/earth-rover/deepgis_vehicle.key'))
        self.declare_parameter('vehicle_id', 'santacruz')
        self.declare_parameter('name', 'Earth Rover')
        self.declare_parameter('platform', 'rover')
        self.declare_parameter('site', '')
        self.declare_parameter('mavros_namespace', '/mavros')
        self.declare_parameter('rate_hz', 1.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.url = p('deepgis_url').rstrip('/') + '/label/api/vehicles/pose/'
        self.vehicle = {'vehicle': p('vehicle_id'), 'name': p('name'), 'platform': p('platform'),
                        'kind': 'field'}
        if p('site'):
            self.vehicle['site'] = p('site')
        with open(os.path.expanduser(p('key_file'))) as f:
            key = f.read().strip()
        self.http = requests.Session()
        self.http.headers.update({'Authorization': f'Bearer {key}'})

        self.fix = None
        self.fix_t = 0.0
        self.heading = None
        self.speed = None
        self.armed = None
        self.failures = 0

        ns = p('mavros_namespace').rstrip('/')
        self.create_subscription(NavSatFix, f'{ns}/global_position/raw/fix', self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Float64, f'{ns}/global_position/compass_hdg', self.on_heading, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, f'{ns}/local_position/velocity_local', self.on_velocity,
                                 qos_profile_sensor_data)
        self.create_subscription(State, f'{ns}/state', self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0 / max(0.1, float(p('rate_hz'))), self.publish_pose)
        self.get_logger().info(f'Posting {self.vehicle["vehicle"]} to {self.url}')

    def on_fix(self, msg):
        if msg.status.status >= 0 and math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.fix, self.fix_t = msg, time.monotonic()

    def on_heading(self, msg):
        self.heading = float(msg.data)

    def on_velocity(self, msg):
        v = msg.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def on_state(self, msg):
        self.armed = bool(msg.armed)

    def publish_pose(self):
        if self.fix is None or time.monotonic() - self.fix_t > FRESH_SEC:
            return
        pose = {**self.vehicle, 'lat': self.fix.latitude, 'lon': self.fix.longitude}
        if math.isfinite(self.fix.altitude):
            pose['alt_amsl'] = self.fix.altitude
        if self.heading is not None and math.isfinite(self.heading):
            pose['heading_deg'] = self.heading
        if self.speed is not None and math.isfinite(self.speed):
            pose['speed_mps'] = self.speed
        if self.armed is not None:
            pose['armed'] = self.armed
        try:
            r = self.http.post(self.url, json=pose, timeout=3.0)
        except requests.RequestException as e:
            self._failed(f'{type(e).__name__}: {e}')
            return
        if r.status_code == 200:
            if self.failures:
                self.get_logger().info('DeepGIS accepting poses again')
            self.failures = 0
        else:
            self._failed(f'HTTP {r.status_code}: {r.text[:200]}')

    def _failed(self, why):
        self.failures += 1
        if self.failures in (1, 10) or self.failures % 60 == 0:
            self.get_logger().warn(f'Pose not accepted ({self.failures}x): {why}')


def main(args=None):
    rclpy.init(args=args)
    node = DeepGISPosePublisher()
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
