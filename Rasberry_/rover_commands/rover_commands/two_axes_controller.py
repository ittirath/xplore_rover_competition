import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import serial
import time


# Constants
SERIAL_PORT = '/dev/ttyUSB0'  # Update with your Arduino's serial port
BAUD_RATE = 115200



class MotorSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('motor_speed_subscriber')
        # Subscriptions
        self.left_sub = self.create_subscription(
            Int16,
            'left_wheel',
            self.left_callback,
            10
        )
        self.right_sub = self.create_subscription(
            Int16,
            'right_wheel',
            self.right_callback,
            10
        )

        self.latest_left = None
        self.latest_right = None
        self.get_logger().info("two wheel Subscriber established")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Allow time for the Arduino to reset (run setup)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial connection established")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.ser = None

    def left_callback(self, msg):
        self.latest_left = msg.data
        self.try_send_to_arduino()

    def right_callback(self, msg):
        self.latest_right = msg.data
        self.try_send_to_arduino()

    def try_send_to_arduino(self):
        if self.latest_left is None or self.latest_right is None:
            return

        self.get_logger().info(f"motor_speed_left = {self.latest_left}, motor_speed_right = {self.latest_right}")

        if self.ser:
            message = f"{self.latest_right},{self.latest_left}\n"
            self.ser.write(message.encode('utf-8'))
            print(f"Sent: {message.strip()}")

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
