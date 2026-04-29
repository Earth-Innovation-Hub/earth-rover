#!/usr/bin/env python3
"""ROS 2 node: read lines from a USB serial laser ranger and publish parsed range."""

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64, String


class LaserRangerNode(Node):
    def __init__(self):
        super().__init__('laser_ranger')

        self.declare_parameter('serial_device', '')
        self.declare_parameter(
            'serial_device_default',
            '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO01SSV5-if00-port0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('topic_raw', 'serial_data')
        self.declare_parameter('topic_distance', 'laser_distance')

        device = self.get_parameter('serial_device').get_parameter_value().string_value
        if not device.strip():
            device = self.get_parameter('serial_device_default').get_parameter_value().string_value

        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        if period <= 0.0:
            period = 0.1

        self._pub_raw = self.create_publisher(
            String, self.get_parameter('topic_raw').get_parameter_value().string_value, 10)
        self._pub_distance = self.create_publisher(
            Float64,
            self.get_parameter('topic_distance').get_parameter_value().string_value,
            10)

        try:
            self._serial = serial.Serial(device, baudrate=baud, timeout=period)
        except (serial.SerialException, OSError, ValueError) as e:
            self.get_logger().fatal(f'Failed to open serial {device!r}: {e}')
            raise

        self.get_logger().info(
            f'Laser ranger serial open {device!r} @ {baud} baud; '
            f'raw topic {self._pub_raw.topic_name}, distance {self._pub_distance.topic_name}')

        self.create_timer(period, self._read_and_publish)

    def _read_and_publish(self):
        try:
            raw = self._serial.readline()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        try:
            line = raw.decode('utf-8', errors='replace').strip()
        except Exception as e:
            self.get_logger().warn(f'Decode error: {e}')
            return

        tokens = line.split()
        if not tokens:
            self.get_logger().warn('Empty line from laser ranger')
            return

        first = tokens[0]
        msg_raw = String()
        msg_raw.data = first
        self._pub_raw.publish(msg_raw)

        try:
            value = float(first)
        except ValueError:
            self.get_logger().debug(f'Non-numeric range token: {first!r}')
            return

        msg_d = Float64()
        msg_d.data = value
        self._pub_distance.publish(msg_d)

    def destroy_node(self):
        if hasattr(self, '_serial') and self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaserRangerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
