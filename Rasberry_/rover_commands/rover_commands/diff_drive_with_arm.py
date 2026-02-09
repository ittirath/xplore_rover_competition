import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

import serial
import time


# Constants
SERIAL_PORT = '/dev/ttyUSB0'  # Update with your Arduino's serial port
BAUD_RATE = 115200
WHEELBASE = 0.5  # Distance between left and right wheels (METERS)
WHEEL_RADIUS = 0.1  # Radius of the wheels (METERS)
MAX_SPEED = 255  # Maximum motor speed

class MotorSpeedSubscriber(Node):
    def __init__(self):

        self.arm_command = 0
        self.gripper_command = 0

        super().__init__('motor_commands')

        self.arm_sub = self.create_subscription(Int8,'arm_cmd',self.listener_callback_arm,10)
        self.get_logger().info("arm_subscriber established")

        self.gripper_sub = self.create_subscription(Int8,'gripper_cmd',self.listener_callback_gripper,10)
        self.get_logger().info("gripper_subscriber established")

        self.wheels_sub = self.create_subscription(Twist,'cmd_vel',self.listener_callback_wheels,10)
        self.get_logger().info("motor_speed subscriber established")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Allow time for the Arduino to reset (run setup)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.ser = None

    def listener_callback_arm(self, msg):
        self.arm_command = msg.data

    def listener_callback_gripper(self, msg):
        self.gripper_command = msg.data

    def listener_callback_wheels(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute wheel velocities
        v_r = (linear_x + (angular_z * WHEELBASE / 2))/WHEEL_RADIUS
        v_l = (linear_x - (angular_z * WHEELBASE / 2))/WHEEL_RADIUS

        # Scale to motor speed
        motor_speed_right = int(max(min(v_r * MAX_SPEED, MAX_SPEED), -MAX_SPEED))
        motor_speed_left = int(max(min(v_l * MAX_SPEED, MAX_SPEED), -MAX_SPEED))

        self.get_logger().info(f"motor_speed_left = {motor_speed_left}, motor_speed_right = {motor_speed_right}")

        message = f"{motor_speed_left},{motor_speed_right},{self.arm_command},{self.gripper_command}\n"

        self.ser.write(message.encode('utf-8'))
        print(f"Sent: {message.strip()}")

        # Optionally, read feedback from the Arduino
        line = self.ser.readline().decode('utf-8').rstrip()
        if line:
            print(f"Arduino says: {line}")

def main(args=None):
    rclpy.init(args=args)   
    node = MotorSpeedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
