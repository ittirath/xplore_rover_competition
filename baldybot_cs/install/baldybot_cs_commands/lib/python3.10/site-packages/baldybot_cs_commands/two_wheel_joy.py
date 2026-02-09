#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

MAX_SPEED = 255  # Maximum motor speed


class MotorJoyController(Node):
    def __init__(self):
        super().__init__('motor_joy_controller')

        # Create publishers for each motor
        self.motor1_pub = self.create_publisher(Int16, '/right_wheel', 10)
        self.motor2_pub = self.create_publisher(Int16, '/left_wheel', 10)
        self.get_logger().info('wheel publishers created')

        # Subscribe to the Joy messages
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,10)
        self.get_logger().info('joy subscriber created')

    def joy_callback(self, joy_msg):

        left_cmd = int(joy_msg.axes[1] * MAX_SPEED)
        right_cmd = int(joy_msg.axes[4] * MAX_SPEED)

        # Publish to each motor command topic
        self.get_logger().info(f'Publishing left: {left_cmd}, right: {right_cmd}')
        self.motor1_pub.publish(Int16(data=left_cmd))
        self.motor2_pub.publish(Int16(data=right_cmd))
        

def main(args=None):
    rclpy.init(args=args)
    node = MotorJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
