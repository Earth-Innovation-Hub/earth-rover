#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import seabreeze.spectrometers as sb
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'spectrometer', 10)
        self.spec = sb.Spectrometer.from_serial_number()
        self.integration_time = 500000
        self.spec.integration_time_micros(self.integration_time)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

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
