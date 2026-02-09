import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

SERIAL_PORT = '/dev/ttyUSB0'  # Update with your Arduino's serial port
BAUD_RATE = 115200
# SLAVE_ADDR_LEFT = 0x01
# SLAVE_ADDR_RIGHT = 0x02

class MotorSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('motor_speed_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.get_logger().info("subscriber established")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
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
        # When linear_x is near zero, we perform an in-place turn.
        if linear_x <= 0.001:
            if angular_z > 0:
                # Turning left in place
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = -3 * motor_speed_step
                self.get_logger().info("Turning in place: Left ")
            elif angular_z < 0:
                # Turning right in place
                motor_speed_right = -3 * motor_speed_step
                motor_speed_left = 3 * motor_speed_step
                self.get_logger().info("Turning in place: Right ")
        # === Moving Forward ===
        # When there's negligible angular input.
        elif abs(angular_z) < 0.1:
            if 0.009 < linear_x < 0.011:  # intended as 0.01
                motor_speed_right = motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at low speed ")
            elif 0.019 < linear_x < 0.021:  # intended as 0.02
                motor_speed_right = 2 * motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at medium speed ")
            elif linear_x >= 0.03:
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = motor_speed_right
                self.get_logger().info("Moving at max speed ")

        # === Turning while moving forward ===
        # We stop moving forward and turn 
        else:
            if angular_z >= 0.1:
                motor_speed_right = 3 * motor_speed_step
                motor_speed_left = 0
                self.get_logger().info("Turning left ")
            elif angular_z <= -0.1:
                motor_speed_right = 0
                motor_speed_left = 3 * motor_speed_step
                self.get_logger().info("Turning right ")

        self.get_logger().info(f"motor_speed_right = {motor_speed_right}, motor_speed_left = {motor_speed_left}")
       
        # slave_right_speed = str(SLAVE_ADDR_RIGHT) + str(motor_speed_right) 
        # motor_right_pmw = str(slave_right_speed).encode('utf-8') + b'\n'
        # self.ser.write(motor_right_pmw)
        # print(f"Sent to Arduino right: {motor_speed_right}")
        
        # motor_left_pmw = str(motor_speed_right).encode('utf-8') + b'\n'
        # self.ser.write(motor_left_pmw)
        # print(f"Sent to Arduino left: {motor_left_pmw}")

        #motor_speed_right_pmw = str(motor_speed_right).encode('utf-8') + b'\n'
        #self.ser.write(motor_speed_right_pmw)
        message = f"{motor_speed_left},{motor_speed_right}\n"
        self.ser.write(message.encode('utf-8'))
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
