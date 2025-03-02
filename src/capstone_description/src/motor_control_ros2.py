#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import sys
import termios
import tty
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

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
        
        # Create a publisher for status messages (optional)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.status_publisher = self.create_publisher(
            String, 
            'robot_status', 
            qos_profile
        )
        
        # Connect to Arduino
        self.connect_to_serial()
        
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
            self.get_logger().error('Is the Arduino connected and the port correct?')
    
    def send_command(self, command):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(command.encode())
                self.current_command = command
                self.get_logger().info(f'Sent command: {command}')
                
                # Publish status message (optional)
                msg = String()
                msg.data = f'Command: {command}'
                self.status_publisher.publish(msg)
                
                # Read response from Arduino
                response = self.serial_connection.readline().decode().strip()
                if response:
                    self.get_logger().info(f'Arduino response: {response}')
                    
            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
        else:
            self.get_logger().warn('Serial connection not available')
    
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
    
    def shutdown(self):
        self.running = False
        if self.serial_connection and self.serial_connection.is_open:
            # Stop motors before closing
            self.send_command('S')
            # Stop pump if active
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
