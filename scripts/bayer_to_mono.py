#!/usr/bin/env python3
"""Bayer-to-mono relay node.

Subscribes to a `sensor_msgs/Image` topic carrying raw Bayer pixels
(default encoding ``bayer_gbrg8`` -- matches the FLIR Grasshopper
``/stereo/right/image_raw`` stream the rover records into rosbags) and
republishes the same frame as ``mono8`` so it can be fed straight into the
ORB-SLAM3 monocular node, which doesn't speak Bayer natively.

Header timestamp and `frame_id` are preserved so SLAM, downstream tools
and RVIZ stay in lock-step with the original capture clock.

Examples:
    # Default plumbing for the right Grasshopper:
    ros2 run earth_rover bayer_to_mono.py

    # Custom topics + downscale (helpful for offline replay on big frames):
    ros2 run earth_rover bayer_to_mono.py \
        --ros-args -p input_topic:=/stereo/right/image_raw \
                   -p output_topic:=/stereo/right/image_mono \
                   -p downscale:=2
"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


_BAYER_DECODE = {
    "bayer_gbrg8": cv2.COLOR_BayerGB2GRAY,
    "bayer_grbg8": cv2.COLOR_BayerGR2GRAY,
    "bayer_rggb8": cv2.COLOR_BayerRG2GRAY,
    "bayer_bggr8": cv2.COLOR_BayerBG2GRAY,
}


class BayerToMono(Node):
    def __init__(self) -> None:
        super().__init__("bayer_to_mono")

        self.input_topic = self.declare_parameter(
            "input_topic", "/stereo/right/image_raw"
        ).get_parameter_value().string_value
        self.output_topic = self.declare_parameter(
            "output_topic", "/stereo/right/image_mono"
        ).get_parameter_value().string_value
        self.downscale = int(self.declare_parameter(
            "downscale", 1
        ).get_parameter_value().integer_value)
        self.expected_encoding = self.declare_parameter(
            "expected_encoding", ""
        ).get_parameter_value().string_value

        if self.downscale < 1:
            self.downscale = 1

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            Image, self.input_topic, self._cb, qos
        )
        self.pub = self.create_publisher(Image, self.output_topic, qos)

        self._warned_unknown_encoding = False
        self.get_logger().info(
            "bayer_to_mono: %s (bayer*) -> %s (mono8) downscale=%d",
            self.input_topic, self.output_topic, self.downscale,
        )

    def _decode(self, msg: Image) -> Optional[np.ndarray]:
        enc = msg.encoding.lower().strip()
        if self.expected_encoding and enc != self.expected_encoding:
            if not self._warned_unknown_encoding:
                self.get_logger().warn(
                    "got encoding %r; expected %r (continuing best-effort)",
                    enc, self.expected_encoding,
                )
                self._warned_unknown_encoding = True

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            buf = buf.reshape((msg.height, msg.step))
        except ValueError:
            self.get_logger().error(
                "bad image dims: H=%d W=%d step=%d data_len=%d",
                msg.height, msg.width, msg.step, len(msg.data),
            )
            return None
        buf = buf[:, : msg.width]

        if enc in _BAYER_DECODE:
            return cv2.cvtColor(buf, _BAYER_DECODE[enc])
        if enc in ("mono8", "8uc1"):
            return buf.copy()
        if enc in ("rgb8", "bgr8"):
            channels = 3
            try:
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, channels)
                )
            except ValueError:
                return None
            code = cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY
            return cv2.cvtColor(rgb, code)

        if not self._warned_unknown_encoding:
            self.get_logger().warn(
                "unsupported encoding %r; passing the first plane through",
                enc,
            )
            self._warned_unknown_encoding = True
        return buf.copy()

    def _cb(self, msg: Image) -> None:
        gray = self._decode(msg)
        if gray is None:
            return
        if self.downscale > 1:
            gray = cv2.resize(
                gray,
                (gray.shape[1] // self.downscale, gray.shape[0] // self.downscale),
                interpolation=cv2.INTER_AREA,
            )

        out = Image()
        out.header = msg.header
        out.height, out.width = gray.shape[:2]
        out.encoding = "mono8"
        out.is_bigendian = 0
        out.step = out.width
        out.data = gray.tobytes()
        self.pub.publish(out)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = BayerToMono()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
