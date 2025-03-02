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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        
        # Create a publisher for status messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Connect to Arduino
        self.connect_to_serial()
        
        # Start keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener_thread)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Timer for odometry updates
        self.create_timer(self.dt, self.publish_odometry)
        
        self.get_logger().info('Robot controller initialized. Use keys to control the robot.')
        self.get_logger().info('Controls: W/A/S/D = Move, Space = Stop, P = Pump On, O = Pump Off, Q = Quit')

    def connect_to_serial(self):
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')

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
    
    # def cmd_vel_callback(self, msg: Twist):
    #     self.linear_velocity = msg.linear.x
    #     self.angular_velocity = msg.angular.z
    
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
        # Calculate new position
        delta_x = self.linear_velocity * self.dt * math.cos(self.theta)
        delta_y = self.linear_velocity * self.dt * math.sin(self.theta)
        delta_theta = self.angular_velocity * self.dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity
        # to check if working use debug statement below
        # self.get_logger().info(f"Odometry: x={self.x}, y={self.y}, theta={self.theta}")
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
