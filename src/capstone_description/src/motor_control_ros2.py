#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import sys
import termios
import tty
import threading
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Serial port setup
        self.serial_port = '/dev/ttyUSB0'  # Change this to match your Arduino port
        self.baud_rate = 115200
        self.serial_connection = None
        
        # Robot state
        self.current_command = 'S'  # Start with motors stopped
        self.pumpActive = False
        self.running = True
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.dt = 0.1  # Time step (100ms)
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.wheel_radius = 0.025  
        self.wheel_base = 0.3    
        
        # Create a publisher for status messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Connect to Arduino
        self.connect_to_serial()
        
        # Timer for odometry updates
        self.create_timer(self.dt, self.publish_odometry)

        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener_thread)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('Robot controller initialized. Use keys to control the robot.')
        self.get_logger().info('Controls: W/A/S/D = Move, Space = Stop, P = Pump On, O = Pump Off, Q = Quit')

    def connect_to_serial(self):
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')

    def read_encoder_data(self):
        """Read encoder data from Arduino."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line.startswith("ENCODER"):
                        # Parse the encoder data from the Arduino
                        parts = line.split(',')
                        for part in parts:
                            if part.startswith("ENCODER_LEFT"):
                                self.left_encoder_count = int(part.split(':')[1])
                            elif part.startswith("ENCODER_RIGHT"):
                                self.right_encoder_count = int(part.split(':')[1])
                        return True
                return False
            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
                return False
        return False

    def send_command(self, command):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode())
                self.current_command = command
                self.get_logger().info(f'Sent command: {command}')
                
                msg = String()
                msg.data = f'Command: {command}'
                self.status_publisher.publish(msg)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
        # to check if working use debug statement below
        # self.get_logger().info(f"Sent command to Arduino: {command}")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def keyboard_listener_thread(self):
        self.get_logger().info('Keyboard listener started')
        while self.running:
            key = self.get_key()
            if key.lower() == 'w':
                self.send_command('F')  # Forward
            elif key.lower() == 's':
                self.send_command('B')  # Backward
            elif key.lower() == 'a':
                self.send_command('L')  # Left
            elif key.lower() == 'd':
                self.send_command('R')  # Right
            elif key == ' ':
                self.send_command('S')  # Stop
            elif key.lower() == 'p':
                self.send_command('P')  # Pump on
                self.pumpActive = True
            elif key.lower() == 'o':
                self.send_command('O')  # Pump off
                self.pumpActive = False
            elif key.lower() == 'q' or key == '\x03':  # q or Ctrl+C
                self.running = False
                break
    
    def cmd_vel_callback(self, msg: Twist):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

        # Determine movement direction based on cmd_vel input
        if self.linear_velocity > 0:
            self.send_command('F')  # Forward
        elif self.linear_velocity < 0:
            self.send_command('B')  # Backward
        elif self.angular_velocity > 0:
            self.send_command('L')  # Left (Rotate CCW)
        elif self.angular_velocity < 0:
            self.send_command('R')  # Right (Rotate CW)
        else:
            self.send_command('S')  # Stop

    def publish_odometry(self):
        # Read encoder data from Arduino
        self.read_encoder_data()

        # Calculate displacement and orientation change based on encoder counts
        # Convert encoder counts to distance traveled (assuming each encoder tick corresponds to a known distance)
        left_distance = self.left_encoder_count * 2 * math.pi * self.wheel_radius / 360  # Convert encoder count to distance
        right_distance = self.right_encoder_count * 2 * math.pi * self.wheel_radius / 360

        # Calculate the robot's change in position and orientation
        delta_d = (left_distance + right_distance) / 2  # Average of left and right wheel displacements
        delta_theta = (right_distance - left_distance) / self.wheel_base  # Difference between wheel displacements
        delta_x = delta_d * math.cos(self.theta + delta_theta / 2)  # X displacement
        delta_y = delta_d * math.sin(self.theta + delta_theta / 2)  # Y displacement

        # Update position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = delta_d / self.dt  # Linear velocity (distance/time)
        odom_msg.twist.twist.angular.z = delta_theta / self.dt  # Angular velocity (angle/time)

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

    def shutdown(self):
        self.running = False
        if self.serial_connection and self.serial_connection.is_open:
            self.send_command('S')
            if self.pumpActive:
                self.send_command('O')
            self.serial_connection.close()
            self.get_logger().info('Serial connection closed')


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.shutdown()
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# import serial
# import time
# import sys
# import termios
# import tty
# import threading
# import math
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry

# class RobotController(Node):
#     def __init__(self):
#         super().__init__('robot_controller')
        
#         # Serial port setup
#         self.serial_port = '/dev/ttyUSB1'  # Change this to match your Arduino port
#         self.baud_rate = 115200
#         self.serial_connection = None
        
#         # Robot state
#         self.current_command = 'S'  # Start with motors stopped
#         self.pumpActive = False
#         self.running = True
#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0
#         self.dt = 0.1  # Time step (100ms)
#         self.left_encoder_count = 0
#         self.right_encoder_count = 0
#         self.wheel_radius = 0.025  
#         self.wheel_base = 0.3    
        
#         # Create a publisher for status messages
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)
#         self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
#         # Subscribe to cmd_vel
#         self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
#         # Connect to Arduino
#         self.connect_to_serial()
        
#         # Timer for odometry updates
#         self.create_timer(self.dt, self.publish_odometry)

#         # Start keyboard listener in a separate thread
#         self.keyboard_thread = threading.Thread(target=self.keyboard_listener_thread)
#         self.keyboard_thread.daemon = True
#         self.keyboard_thread.start()
        
#         # Timer for odometry updates
#         self.create_timer(self.dt, self.publish_odometry)
        
#         self.get_logger().info('Robot controller initialized. Use keys to control the robot.')
#         self.get_logger().info('Controls: W/A/S/D = Move, Space = Stop, P = Pump On, O = Pump Off, Q = Quit')

#     def connect_to_serial(self):
#         try:
#             self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             time.sleep(2)  # Wait for Arduino to reset
#             self.get_logger().info(f'Connected to {self.serial_port}')
#         except serial.SerialException as e:
#             self.get_logger().error(f'Failed to connect to serial port: {e}')

#     def read_encoder_data(self):
#         """Read encoder data from Arduino."""
#         if self.serial_connection and self.serial_connection.is_open:
#             try:
#                 line = self.serial_connection.readline().decode('utf-8').strip()
#                 if line.startswith("ENCODER"):
#                     # Parse the encoder data from the Arduino
#                     parts = line.split(',')
#                     for part in parts:
#                         if part.startswith("ENCODER_LEFT"):
#                             self.left_encoder_count = int(part.split(':')[1])
#                         elif part.startswith("ENCODER_RIGHT"):
#                             self.right_encoder_count = int(part.split(':')[1])
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Serial communication error: {e}')


#     def send_command(self, command):
#         if self.serial_connection and self.serial_connection.is_open:
#             try:
#                 self.serial_connection.write(command.encode())
#                 self.current_command = command
#                 self.get_logger().info(f'Sent command: {command}')
                
#                 msg = String()
#                 msg.data = f'Command: {command}'
#                 self.status_publisher.publish(msg)
#             except serial.SerialException as e:
#                 self.get_logger().error(f'Serial communication error: {e}')
#         # to check if working use debug statement below
#         # self.get_logger().info(f"Sent command to Arduino: {command}")

#     def get_key(self):
#         fd = sys.stdin.fileno()
#         old_settings = termios.tcgetattr(fd)
#         try:
#             tty.setraw(fd)
#             key = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return key
    
#     def keyboard_listener_thread(self):
#         self.get_logger().info('Keyboard listener started')
#         while self.running:
#             key = self.get_key()
#             if key.lower() == 'w':
#                 self.send_command('F')  # Forward
#             elif key.lower() == 's':
#                 self.send_command('B')  # Backward
#             elif key.lower() == 'a':
#                 self.send_command('L')  # Left
#             elif key.lower() == 'd':
#                 self.send_command('R')  # Right
#             elif key == ' ':
#                 self.send_command('S')  # Stop
#             elif key.lower() == 'p':
#                 self.send_command('P')  # Pump on
#                 self.pumpActive = True
#             elif key.lower() == 'o':
#                 self.send_command('O')  # Pump off
#                 self.pumpActive = False
#             elif key.lower() == 'q' or key == '\x03':  # q or Ctrl+C
#                 self.running = False
#                 break
    
#     # def cmd_vel_callback(self, msg: Twist):
#     #     self.linear_velocity = msg.linear.x
#     #     self.angular_velocity = msg.angular.z
    
#     def cmd_vel_callback(self, msg: Twist):
#         self.linear_velocity = msg.linear.x
#         self.angular_velocity = msg.angular.z

#         # Determine movement direction based on cmd_vel input
#         if self.linear_velocity > 0:
#             self.send_command('F')  # Forward
#         elif self.linear_velocity < 0:
#             self.send_command('B')  # Backward
#         elif self.angular_velocity > 0:
#             self.send_command('L')  # Left (Rotate CCW)
#         elif self.angular_velocity < 0:
#             self.send_command('R')  # Right (Rotate CW)
#         else:
#             self.send_command('S')  # Stop

#     # def publish_odometry(self):
#     #     # Calculate new position
#     #     delta_x = self.linear_velocity * self.dt * math.cos(self.theta)
#     #     delta_y = self.linear_velocity * self.dt * math.sin(self.theta)
#     #     delta_theta = self.angular_velocity * self.dt
        
#     #     self.x += delta_x
#     #     self.y += delta_y
#     #     self.theta += delta_theta
        
#     #     # Publish odometry
#     #     odom_msg = Odometry()
#     #     odom_msg.header.stamp = self.get_clock().now().to_msg()
#     #     odom_msg.header.frame_id = 'odom'
#     #     odom_msg.child_frame_id = 'base_link'
#     #     odom_msg.pose.pose.position.x = self.x
#     #     odom_msg.pose.pose.position.y = self.y
#     #     odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#     #     odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
#     #     odom_msg.twist.twist.linear.x = self.linear_velocity
#     #     odom_msg.twist.twist.angular.z = self.angular_velocity
#     #     # to check if working use debug statement below
#     #     # self.get_logger().info(f"Odometry: x={self.x}, y={self.y}, theta={self.theta}")
#     #     self.odom_publisher.publish(odom_msg)


#     def publish_odometry(self):
#         # Read encoder data from Arduino
#         self.read_encoder_data()

#         # Calculate displacement and orientation change based on encoder counts
#         # Convert encoder counts to distance traveled (assuming each encoder tick corresponds to a known distance)
#         left_distance = self.left_encoder_count * 2 * math.pi * self.wheel_radius / 360  # Convert encoder count to distance (example)
#         right_distance = self.right_encoder_count * 2 * math.pi * self.wheel_radius / 360

#         # Calculate the robot's change in position and orientation
#         delta_d = (left_distance + right_distance) / 2  # Average of left and right wheel displacements
#         delta_theta = (right_distance - left_distance) / self.wheel_base  # Difference between wheel displacements
#         delta_x = delta_d * math.cos(self.theta + delta_theta / 2)  # X displacement
#         delta_y = delta_d * math.sin(self.theta + delta_theta / 2)  # Y displacement

#         # Update position and orientation
#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta
        
#         # Publish the odometry message
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
#         odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
#         odom_msg.twist.twist.linear.x = delta_d / self.dt  # Linear velocity (distance/time)
#         odom_msg.twist.twist.angular.z = delta_theta / self.dt  # Angular velocity (angle/time)

#         # Publish the odometry message
#         self.odom_publisher.publish(odom_msg)

#     def shutdown(self):
#         self.running = False
#         if self.serial_connection and self.serial_connection.is_open:
#             self.send_command('S')
#             if self.pumpActive:
#                 self.send_command('O')
#             self.serial_connection.close()
#             self.get_logger().info('Serial connection closed')


# def main(args=None):
#     rclpy.init(args=args)
#     robot_controller = RobotController()
#     try:
#         rclpy.spin(robot_controller)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         robot_controller.shutdown()
#         robot_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


