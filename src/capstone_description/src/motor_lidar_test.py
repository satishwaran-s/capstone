import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import serial
import sys
import termios
import tty
import time
import math

# Serial connection to Arduino (Change to your Arduino's port)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Motor control commands
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Encoder data
        self.encoder_count_A = 0
        self.encoder_count_B = 0
        self.last_time = time.time()

        # Set motor speed (PWM value)
        self.motor_speed = 150
        self.speed_step = 25

        # Pump control variables
        self.pump_active = False
        self.pump_pin_1 = 5
        self.pump_pin_2 = 4
        self.pump_pwm = 3

        self.get_logger().info("Keyboard control active. Use WASD to move, + or - to adjust speed, P/O to control pump.")

    def send_command(self, cmd):
        """Send command to Arduino via serial."""
        ser.write(cmd.encode())  # Send command to Arduino
        self.get_logger().info(f"Sent: {cmd}")

    def cmd_vel_callback(self, msg):
        """Callback for receiving cmd_vel (twist) messages."""
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        
        if linear_speed > 0:
            self.send_command('F')  # Move Forward
        elif linear_speed < 0:
            self.send_command('B')  # Move Backward
        if angular_speed > 0:
            self.send_command('R')  # Turn Right
        elif angular_speed < 0:
            self.send_command('L')  # Turn Left

    def get_key(self):
        """Get a single key press from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def update_odometry(self):
        """Update the odometry using encoder counts."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Calculate speeds based on encoder counts (wheel rotations)
        speed_a = (self.encoder_count_A / 360.0) * 60.0 / dt
        speed_b = (self.encoder_count_B / 360.0) * 60.0 / dt
        self.encoder_count_A = 0
        self.encoder_count_B = 0

        # Calculate the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # You can populate position, orientation, etc. here.
        odom_msg.pose.pose.position.x = 0  # Placeholder
        odom_msg.pose.pose.position.y = 0  # Placeholder
        odom_msg.pose.pose.orientation.z = 0  # Placeholder

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def move_forward(self):
        self.send_command('F')

    def move_backward(self):
        self.send_command('B')

    def turn_left(self):
        self.send_command('L')

    def turn_right(self):
        self.send_command('R')

    def stop_motors(self):
        self.send_command('S')

    def increase_speed(self):
        self.motor_speed = min(self.motor_speed + self.speed_step, 255)
        self.get_logger().info(f"Speed Increased: {self.motor_speed}")

    def decrease_speed(self):
        self.motor_speed = max(self.motor_speed - self.speed_step, 0)
        self.get_logger().info(f"Speed Decreased: {self.motor_speed}")

    def start_pump(self):
        self.pump_active = True
        self.send_command('P')
        self.get_logger().info("Pump Activated")

    def stop_pump(self):
        self.pump_active = False
        self.send_command('O')
        self.get_logger().info("Pump Stopped")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    try:
        while True:
            key = node.get_key()
            if key.lower() == 'w':  # Move Forward
                node.move_forward()
            elif key.lower() == 's':  # Move Backward
                node.move_backward()
            elif key.lower() == 'a':  # Turn Left
                node.turn_left()
            elif key.lower() == 'd':  # Turn Right
                node.turn_right()
            elif key == ' ':  # Stop Motors (spacebar)
                node.stop_motors()
            elif key == '+':  # Increase speed
                node.increase_speed()
            elif key == '-':  # Decrease speed
                node.decrease_speed()
            elif key == 'p':  # Start Pump
                node.start_pump()
            elif key == 'o':  # Stop Pump
                node.stop_pump()
            elif key == '\x03':  # Ctrl+C to exit
                break
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
