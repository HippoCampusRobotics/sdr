#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sdr_msgs.msg import Rss

from sdr.dvb_t import NooElec


class SdrNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.dvb_t_dongle = NooElec()

        self.sdr_pub = self.create_publisher(Rss,
                                             'max_rss_value',
                                             qos_profile=1)

        self.timer = self.create_timer(timer_period_sec=(1 / 100.0),
                                       callback=self.read_samples)

    def read_samples(self):

        exp_peak_freq = 434.168e6  # around which frequency we search for peak

        frequency, pxx_density = self.dvb_t_dongle.get_rss_peak(exp_peak_freq)
        msg = Rss()
        msg.rss_peak = pxx_density
        msg.exact_frequency = frequency
        msg.frequency = exp_peak_freq
        msg.header.stamp = self.get_clock().now().to_msg()
        self.sdr_pub.publish(msg)


def main():
    rclpy.init()
    node = SdrNode("sdr_node")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
