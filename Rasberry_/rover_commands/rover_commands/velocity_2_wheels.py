import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'  # Update with your Arduino's serial port
BAUD_RATE = 115200

class MotorSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('motor_speed_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.get_logger().info("Subscriber established")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Allow time for the Arduino to reset (run setup)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial connection established")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")

    def listener_callback(self, msg):
        motor_speed_step = 85  # 255/3 = 85
        motor_speed_right = 0
        motor_speed_left = 0

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # === Turning in place ===
        if abs(linear_x) < 0.001:
            if angular_z > 0:
                # Turning left in place
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = -3 * motor_speed_step
                self.get_logger().info("Turning in place: Left")
            elif angular_z < 0:
                # Turning right in place
                motor_speed_right = -3 * motor_speed_step
                motor_speed_left = 3 * motor_speed_step
                self.get_logger().info("Turning in place: Right")
        # === Moving Forward ===
        elif abs(angular_z) < 0.1:
            if 0.009 < linear_x < 0.011:  # intended as 0.01
                motor_speed_right = motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at low speed")
            elif 0.019 < linear_x < 0.021:  # intended as 0.02
                motor_speed_right = 2 * motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at medium speed")
            elif linear_x >= 0.03:
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at max speed")
        # === Turning while moving forward ===
        else:
            if angular_z >= 0.1:
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = 0
                self.get_logger().info("Turning left")
            elif angular_z <= -0.1:
                motor_speed_right = 0
                motor_speed_left = 3 * motor_speed_step
                self.get_logger().info("Turning right")

        self.get_logger().info(f"Right: {motor_speed_right}, Left: {motor_speed_left}")

        # Format the message by concatenating the slave address and motor speeds.
        # The message format is: "slave_addr_left:motor_speed_left,slave_addr_right:motor_speed_right\n"
        #message = f"{SLAVE_ADDR_LEFT}:{motor_speed_left},{SLAVE_ADDR_RIGHT}:{motor_speed_right}\n"
        
        #message = f"{SLAVE_ADDR_LEFT}:{motor_speed_left},{SLAVE_ADDR_RIGHT}:{motor_speed_right}" + b'\n'
        #message = f"{SLAVE_ADDR_LEFT}:{motor_speed_left},{SLAVE_ADDR_RIGHT}:{motor_speed_right}\n"
        message = f"{motor_speed_left},{motor_speed_right}\n"

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
