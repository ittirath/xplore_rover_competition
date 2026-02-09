#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyThrottler(Node):
    def __init__(self):
        super().__init__('joy_throttler')
        self.last_msg = None
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Joy, '/joy_throttled', 10)
        self.timer = self.create_timer(0.1, self.publish_throttled)  # 10 Hz

    def joy_callback(self, msg):
        self.last_msg = msg

    def publish_throttled(self):
        if self.last_msg:
            self.pub.publish(self.last_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyThrottler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
