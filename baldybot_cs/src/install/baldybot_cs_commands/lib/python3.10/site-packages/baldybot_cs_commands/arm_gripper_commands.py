#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class MotorJoyController(Node):
    def __init__(self):
        super().__init__('motor_joy_controller')

        # Create publishers for each motor
        self.motor1_pub = self.create_publisher(Int8, '/arm_cmd', 10)
        self.motor2_pub = self.create_publisher(Int8, '/gripper_cmd', 10)
        self.get_logger().info('arm_cmd and gripper_cmd publishers created')

        # Subscribe to the Joy messages
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,10)
        self.get_logger().info('joy subscriber created')

    def joy_callback(self, joy_msg):
        #   button 0 => X
        #   button 1 => O
        #   button 2 => Triangle
        #   button 3 => Square

        motor1_cmd = 0
        motor2_cmd = 0

        # Check each button state
        if joy_msg.buttons[1] == 1:
            motor1_cmd = 1
        elif joy_msg.buttons[3] == 1:
            motor1_cmd = -1

        if joy_msg.buttons[2] == 1:
            motor2_cmd = 1
        elif joy_msg.buttons[0] == 1:
            motor2_cmd = -1

        # Publish to each motor command topic
        self.get_logger().info(f'Publishing arm_cmd: {motor1_cmd}, gripper_cmd: {motor2_cmd}')
        self.motor1_pub.publish(Int8(data=motor1_cmd))
        self.motor2_pub.publish(Int8(data=motor2_cmd))
        

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
