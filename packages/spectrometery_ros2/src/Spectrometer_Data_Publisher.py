#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import seabreeze.spectrometers as sb
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('spectrometer_data_publisher')
        self.declare_parameter('topic', 'spectrometer')
        self.declare_parameter('integration_time_micros', 500_000)
        self.declare_parameter('publish_period_sec', 0.1)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.integration_time = self.get_parameter(
            'integration_time_micros').get_parameter_value().integer_value
        timer_period = self.get_parameter(
            'publish_period_sec').get_parameter_value().double_value
        if timer_period <= 0.0:
            timer_period = 0.1

        self.publisher_ = self.create_publisher(Float64MultiArray, topic, 10)
        self.spec = sb.Spectrometer.from_serial_number()
        self.spec.integration_time_micros(self.integration_time)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Spectrometer publisher: topic={topic!r}, integration={self.integration_time} µs, '
            f'period={timer_period} s')

    def timer_callback(self):
        try:
            intensities = np.array(self.spec.intensities(), dtype=np.double)
            wavelengths = np.array(self.spec.wavelengths(), dtype=np.double)
            data = np.concatenate(([self.integration_time], intensities, wavelengths))

            msg = Float64MultiArray()
            msg.layout.dim = [MultiArrayDimension()]
            msg.layout.dim[0].size = data.size
            msg.layout.dim[0].stride = 1
            msg.layout.dim[0].label = 'spectrometer_data'  # label it as needed
            msg.data = data.tolist()

            self.publisher_.publish(msg)
        except Exception as exception:
            self.get_logger().error(f'Exception in timer_callback: {exception}')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
