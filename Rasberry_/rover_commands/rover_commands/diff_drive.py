import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


# Constants
SERIAL_PORT = '/dev/ttyUSB0'  # Update with your Arduino's serial port
BAUD_RATE = 115200
WHEELBASE = 0.3  # Distance between left and right wheels (METERS)
WHEEL_RADIUS = 0.0375  # Radius of the wheels (METERS)
MAX_SPEED = 255  # Maximum motor speed
# MAX_SPEED = 200  # Maximum motor speed
LINEAR_FACTOR = 0.8  # Linear speed factor
ANGULAR_FACTOR = 1.2  # Angular speed factor




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
            self.ser = None

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute wheel velocities
        v_r = (linear_x*LINEAR_FACTOR + (angular_z*ANGULAR_FACTOR * WHEELBASE / 2))/WHEEL_RADIUS
        v_l = (linear_x*LINEAR_FACTOR - (angular_z* ANGULAR_FACTOR * WHEELBASE / 2))/WHEEL_RADIUS

        # Scale to motor speed
        motor_speed_right = int(max(min(v_r * MAX_SPEED, MAX_SPEED), -MAX_SPEED))
        motor_speed_left = int(max(min(v_l * MAX_SPEED, MAX_SPEED), -MAX_SPEED))

        self.get_logger().info(f"motor_speed_left = {motor_speed_left}, motor_speed_right = {motor_speed_right}")
       
        # slave_right_speed = str(SLAVE_ADDR_RIGHT) + str(motor_speed_right) 
        # motor_right_pmw = str(slave_right_speed).encode('utf-8') + b'\n'
        # self.ser.write(motor_right_pmw)
        # print(f"Sent to Arduino right: {motor_speed_right}")

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
