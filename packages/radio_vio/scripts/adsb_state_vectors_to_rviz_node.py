#!/usr/bin/env python3
"""
Converts ADS-B aircraft state vectors (JSON) to RViz-friendly topics.

Subscribes to ~/aircraft_state_vectors (std_msgs/String, JSON) and publishes:
  - PoseArray: aircraft poses in map frame (trike position + aircraft offset).
  - MarkerArray: velocity arrows and optional ICAO labels.

Expects state vectors in trike-centric ENU; adds trike pose position so all
poses are in the same frame as MAVROS local (frame_id, default "map").
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker


class StateVectorsToRvizNode(Node):
    def __init__(self):
        super().__init__('adsb_state_vectors_to_rviz_node')

        self.declare_parameter('state_vectors_topic', '~/aircraft_state_vectors')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('velocity_arrow_scale', 0.5)   # length per m/s
        self.declare_parameter('velocity_arrow_min_length', 2.0)
        self.declare_parameter('show_labels', True)

        topic = self.get_parameter('state_vectors_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.vel_scale = float(self.get_parameter('velocity_arrow_scale').value)
        self.vel_min = float(self.get_parameter('velocity_arrow_min_length').value)
        self.show_labels = self.get_parameter('show_labels').value

        self._pose_pub = self.create_publisher(PoseArray, '~/aircraft_poses', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/aircraft_velocity_markers', 10)

        self.create_subscription(String, topic, self._callback, 10)

        self.get_logger().info(
            'State vectors to RViz: subscribing to %s, frame_id=%s' % (topic, self.frame_id)
        )

    def _callback(self, msg):
        raw = msg.data or ''
        if not raw.strip().startswith('{'):
            return
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        trike = data.get('trike') or {}
        trike_pose = trike.get('pose') or {}
        trike_pos = trike_pose.get('position') or {}
        tx = float(trike_pos.get('x', 0.0))
        ty = float(trike_pos.get('y', 0.0))
        tz = float(trike_pos.get('z', 0.0))

        aircraft_list = data.get('aircraft') or []
        if not aircraft_list:
            self._pose_pub.publish(self._empty_pose_array())
            self._marker_pub.publish(MarkerArray())
            return

        pose_array = PoseArray()
        pose_array.header.frame_id = self.frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()

        markers = []
        for i, ac in enumerate(aircraft_list):
            icao = (ac.get('icao') or '').strip().upper()
            pose_j = ac.get('pose') or {}
            pos_j = pose_j.get('position') or {}
            ori_j = pose_j.get('orientation') or {}
            twist_j = ac.get('twist') or {}
            lin_j = twist_j.get('linear') or {}

            # Aircraft position in trike-centric ENU; add trike to get map frame
            px = tx + float(pos_j.get('x', 0.0))
            py = ty + float(pos_j.get('y', 0.0))
            pz = tz + float(pos_j.get('z', 0.0))

            p = Pose()
            p.position.x = px
            p.position.y = py
            p.position.z = pz
            p.orientation.x = float(ori_j.get('x', 0.0))
            p.orientation.y = float(ori_j.get('y', 0.0))
            p.orientation.z = float(ori_j.get('z', 0.0))
            p.orientation.w = float(ori_j.get('w', 1.0))
            pose_array.poses.append(p)

            # Velocity arrow: from position, direction = linear velocity
            vx = float(lin_j.get('x', 0.0))
            vy = float(lin_j.get('y', 0.0))
            vz = float(lin_j.get('z', 0.0))
            speed = (vx * vx + vy * vy + vz * vz) ** 0.5
            length = max(self.vel_min, speed * self.vel_scale)
            if speed > 0.1:
                scale = length / speed
                dx = vx * scale
                dy = vy * scale
                dz = vz * scale
            else:
                dx = dy = dz = 0.0

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = pose_array.header.stamp
            arrow.ns = 'velocity'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = length
            arrow.scale.y = 0.15
            arrow.scale.z = 0.1
            arrow.color.r = 0.0
            arrow.color.g = 0.8
            arrow.color.b = 1.0
            arrow.color.a = 0.9
            arrow.points = [
                Point(x=px, y=py, z=pz),
                Point(x=px + dx, y=py + dy, z=pz + dz),
            ]
            markers.append(arrow)

            if self.show_labels and icao:
                label = Marker()
                label.header.frame_id = self.frame_id
                label.header.stamp = pose_array.header.stamp
                label.ns = 'icao'
                label.id = i
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = px
                label.pose.position.y = py
                label.pose.position.z = pz + 50.0
                label.pose.orientation.w = 1.0
                label.scale.z = 30.0
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 1.0
                label.text = icao
                markers.append(label)

        self._pose_pub.publish(pose_array)
        self._marker_pub.publish(MarkerArray(markers=markers))

    def _empty_pose_array(self):
        pa = PoseArray()
        pa.header.frame_id = self.frame_id
        pa.header.stamp = self.get_clock().now().to_msg()
        return pa


def main(args=None):
    rclpy.init(args=args)
    node = StateVectorsToRvizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
