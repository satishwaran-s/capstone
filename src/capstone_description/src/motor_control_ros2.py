import rclpy
from rclpy.node import Node
import serial
import sys
import termios
import tty
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

# # serial connection to arduino
# ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Change to your Arduino's port

# class MotorController(Node):
#     def __init__(self):
#         super().__init__('motor_controller')
#         self.get_logger().info("Keyboard control active. Use WASD to move, Space to stop.")

#     def send_command(self, cmd):
#         ser.write(cmd.encode())  # send command to Arduino
#         self.get_logger().info(f"Sent: {cmd}")

# def get_key():
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(fd)
#         key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorController()

#     try:
#         while True:
#             key = get_key()
#             if key.lower() == 'w':  # forward
#                 node.send_command('F')
#             elif key.lower() == 's':  # backward
#                 node.send_command('B')
#             elif key.lower() == 'a':  # left
#                 node.send_command('L')
#             elif key.lower() == 'd':  # right
#                 node.send_command('R')
#             elif key == ' ':  # stop which is spacebar
#                 node.send_command('S')
#             elif key == '\x03':  # ctrl+c to exit
#                 break
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ser.close()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Setup serial connection to Arduino (change to your actual port)
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        # Initialize scaling factors for speed control
        self.scale_linear = 0.5  # Initial linear speed scaling factor
        self.scale_angular = 0.5  # Initial angular speed scaling factor

        # Subscriber for /joy topic to get joystick inputs
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
    # Extract joystick axes and buttons
        axes = msg.axes
        buttons = msg.buttons

        # Adjust speed scaling using axes[6] (left/right button)
        if axes[6] == 1:  # Right button pressed (increase speed)
            self.scale_linear = min(1.0, self.scale_linear + 0.1)
            self.scale_angular = min(1.0, self.scale_angular + 0.1)
            self.get_logger().info(f"Speed increased: linear={self.scale_linear}, angular={self.scale_angular}")
        elif axes[6] == -1:  # Left button pressed (decrease speed)
            self.scale_linear = max(0.1, self.scale_linear - 0.1)
            self.scale_angular = max(0.1, self.scale_angular - 0.1)
            self.get_logger().info(f"Speed decreased: linear={self.scale_linear}, angular={self.scale_angular}")

        # Stop motion if button[0] (X button) is pressed
        if buttons[0] == 1:
            self.ser.write(b"S\n")  # Send stop command
            self.get_logger().info("Stopping motion")
            return

        # Map axes[0] and axes[1] to linear speed (forward/backward)
        linear_speed = axes[1] * self.scale_linear  # Forward (positive) or backward (negative)
        if linear_speed > 0:
            self.get_logger().info(f"Moving forward with speed {linear_speed}")
            self.ser.write(f"F{int(linear_speed * 255)}\n".encode())  # Send forward command
        elif linear_speed < 0:
            self.get_logger().info(f"Moving backward with speed {linear_speed}")
            self.ser.write(f"R{int(abs(linear_speed) * 255)}\n".encode())  # Send reverse command

        # Map axes[3] and axes[4] to angular speed (left/right)
        angular_speed = axes[3] * self.scale_angular  # Right (positive) or left (negative)
        if angular_speed > 0:
            self.get_logger().info(f"Turning right with speed {angular_speed}")
            self.ser.write(f"T{int(angular_speed * 255)}\n".encode())  # Turn right
        elif angular_speed < 0:
            self.get_logger().info(f"Turning left with speed {angular_speed}")
            self.ser.write(f"T{-int(angular_speed * 255)}\`n".encode())  # Turn left`



def main(args=None):
    rclpy.init(args=args)

    motor_control_node = MotorControlNode()

    rclpy.spin(motor_control_node)

    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

