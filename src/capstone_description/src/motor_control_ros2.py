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
import re
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class SerialMotorController(Node):
    def __init__(self):
        super().__init__("serial_motor_controller")

        # Parameters for testing (constant velocities)
        self.declare_parameter(
            "use_constant_velocity", False
        )  # Changed to False to use encoder data
        self.declare_parameter("constant_vx", 0.2)
        self.declare_parameter("constant_vy", 0.0)
        self.declare_parameter("constant_vth", 0.1)

        # Get parameters
        self.use_constant_velocity = (
            self.get_parameter("use_constant_velocity").get_parameter_value().bool_value
        )
        self.constant_vx = (
            self.get_parameter("constant_vx").get_parameter_value().double_value
        )
        self.constant_vy = (
            self.get_parameter("constant_vy").get_parameter_value().double_value
        )
        self.constant_vth = (
            self.get_parameter("constant_vth").get_parameter_value().double_value
        )

        # Initialize serial connection to Arduino
        self.serial_port = serial.Serial(
            "/dev/ttyUSB1", 115200, timeout=1
        )  # Change to your Arduino port
        self.get_logger().info("Connected to Arduino via Serial")
        self.running = True

        # create a timer for odometry updates which reads every 0.1 seconds which is 100ms cos we need to publish 10hz
        self.odom_timer = self.create_timer(0.1, self.read_encoder_and_publish_odom)

        # ROS2 Publishers and Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, qos_profile
        )

        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Start the keyboard listener thread for manual control
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        # Odometry Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_left_distance = 0.0
        self.last_right_distance = 0.0

    def send_command(self, command):
        # Send command to Arduino
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command}")

    def get_key(self):
        # to get a key press for manual control
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_listener(self):
        # Manual control using keyboard keys
        self.get_logger().info(
            "Use W/A/S/D for movement, P to start pump, O to stop pump, Q to quit"
        )
        while self.running:
            key = self.get_key()
            if key in ["w", "W"]:
                self.send_command("F")  # forward
            elif key in ["s", "S"]:
                self.send_command("B")  # backward
            elif key in ["a", "A"]:
                self.send_command("L")  # left
            elif key in ["d", "D"]:
                self.send_command("R")  # right
            elif key in [" "]:
                self.send_command("S")  # stop
            elif key in ["p", "P"]:
                self.send_command("P")  # pump on
            elif key in ["o", "O"]:
                self.send_command("O")  # pump off
            elif key in ["q", "Q"]:
                self.get_logger().info("Shutting down...")
                self.running = False
                self.send_command("S")  # stop motors before exit
                break

    def cmd_vel_callback(self, msg):
        # Callback to handle cmd_vel messages
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        self.send_command(f"V{linear_speed},{angular_speed}")
        self.get_logger().info(
            f"Received cmd_vel - Linear: {linear_speed}, Angular: {angular_speed}"
        )

    def read_encoder_and_publish_odom(self):
        MAX_DRIFT_THRESHOLD = 0.5  # adjust as needed for resetting odom

        if self.serial_port.in_waiting > 0:
            try:
                data = self.serial_port.readline().decode().strip()
                self.get_logger().info(f"Received serial data: {data}")

                # format for distance which will be read from arduino, format MUST BE THE SAME!
                left_pattern = r"L:(-?[\d.]+)"
                right_pattern = r"R:(-?[\d.]+)"

                left_match = re.search(left_pattern, data)
                right_match = re.search(right_pattern, data)

                if left_match and right_match:
                    left_distance = float(left_match.group(1))
                    right_distance = float(right_match.group(1))
                    self.publish_odometry(left_distance, right_distance)
                    self.get_logger().info(
                        f"Processed distances - Left: {left_distance:.4f}, Right: {right_distance:.4f}"
                    )

                    # # check for drift and reset odometry if needed
                    # # Calculate Euclidean drift distance
                    # drift_distance = math.sqrt(self.x**2 + self.y**2)
                    # if drift_distance > MAX_DRIFT_THRESHOLD:
                    #     self.get_logger().warn(
                    #         f"Odometry drift exceeds threshold. Resetting odometry."
                    #     )
                    #     self.reset_odometry()
                else:
                    self.get_logger().warn(
                        f"Could not extract distances from data: {data}"
                    )
            except Exception as e:
                self.get_logger().error(f"Error processing serial data: {e}")

    def reset_odometry(self):
        # reset the odometry variables to zero
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_distance = 0.0
        self.last_right_distance = 0.0
        self.get_logger().info("Odometry reset.")
    
    def apply_drift_correction(self):
        # Calculate drift
        drift_distance = math.sqrt(self.x**2 + self.y**2)
        
        if drift_distance > self.DRIFT_THRESHOLD:
            # Make correction proportional to how much threshold is exceeded
            excess_drift = drift_distance - self.DRIFT_THRESHOLD
            max_excess = 1.0  # Define a cap for scaling
            
            # Scale correction factor based on excess drift (0.99 to 0.95)
            scale = min(excess_drift / max_excess, 1.0)
            correction_factor = 1.0 - (0.05 * scale)
            
            # Apply correction
            self.x *= correction_factor
            self.y *= correction_factor
            
            self.get_logger().info(
                f"Drift correction applied: {correction_factor:.4f}, "
                f"Position: ({self.x:.2f}, {self.y:.2f})"
            )
    def publish_odometry(self, left_distance, right_distance):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt < 0.001:
            self.get_logger().warn("Time delta too small, skipping odometry update")
            return

        # appy scale factor to left and right distances based on arduino code
        scale_factor = 18.0  # adjust as per arduino readings measurements
        left_distance = left_distance * scale_factor
        right_distance = right_distance * scale_factor

        # calculate change in distance since last update
        delta_left = left_distance - self.last_left_distance
        delta_right = right_distance - self.last_right_distance

        # Save current values for next iteration
        self.last_left_distance = left_distance
        self.last_right_distance = right_distance
        self.last_time = current_time

        # URDF wheel base value
        wheel_base = 0.3  # distance between wheels

        if self.use_constant_velocity:
            # Use constant velocities for testing
            vx = self.constant_vx
            vy = self.constant_vy
            vth = self.constant_vth

            # Simple integration with constant velocities
            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            delta_theta = vth * dt
        else:
            # Calculate the robot's movement from encoder data
            if abs(delta_right - delta_left) < 0.0001:  # Moving straight
                # Simple straight line motion
                delta_x = delta_right * math.cos(self.theta)
                delta_y = delta_right * math.sin(self.theta)
                delta_theta = 0.0
                self.get_logger().debug("Moving straight")
            else:  # Following an arc
                # Arc motion formulas - more accurate for differential drive
                delta_theta = (delta_right - delta_left) / wheel_base

                # Arc motion calculation
                radius = (
                    wheel_base
                    * (delta_left + delta_right)
                    / (2 * (delta_right - delta_left))
                )
                delta_x = radius * (
                    math.sin(self.theta + delta_theta) - math.sin(self.theta)
                )
                delta_y = radius * (
                    -math.cos(self.theta + delta_theta) + math.cos(self.theta)
                )

                # if abs(delta_theta) < 0.0001:
                #     # Handle very small rotation case to avoid division by zero
                #     delta_x = delta_right * math.cos(self.theta)
                #     delta_y = delta_right * math.sin(self.theta)
                # else:
                #     # Full arc calculation
                #     radius = (
                #         wheel_base
                #         * (delta_left + delta_right)
                #         / (2 * (delta_right - delta_left))
                #     )
                #     delta_x = radius * (
                #         math.sin(self.theta + delta_theta) - math.sin(self.theta)
                #     )
                #     delta_y = radius * (
                #         -math.cos(self.theta + delta_theta) + math.cos(self.theta)
                #     )
                # self.get_logger().debug(f"Arc motion: radius={radius:.4f}")

            # For debugging
            self.get_logger().info(
                f"Delta left: {delta_left:.6f}, Delta right: {delta_right:.6f}"
            )
            self.get_logger().info(
                f"Position update: dx={delta_x:.6f}, dy={delta_y:.6f}, dth={delta_theta:.6f}"
            )

        # Update robot position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to prevent drift over time
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # ADD THE DRIFT CORRECTION HERE:
        # Define drift threshold as a class member or constant
        DRIFT_THRESHOLD = 0.5  # Meters - adjust based on your robot and environment
        
        # Calculate current drift from origin
        drift_distance = math.sqrt(self.x**2 + self.y**2)
        
        # Apply correction when drift exceeds threshold
        if drift_distance > DRIFT_THRESHOLD:
            # Start with mild correction
            correction_factor = 0.9  # the lower you go the more drastic corrections you make
            # Apply correction
            self.x *= correction_factor
            self.y *= correction_factor
            
            # Log that correction was applied
            self.get_logger().info(
                f"Drift correction applied: {correction_factor:.4f}, "
                f"Position: ({self.x:.2f}, {self.y:.2f})"
            )

            self.get_logger().info(
                f"New position: x={self.x:.6f}, y={self.y:.6f}, theta={self.theta:.6f}"
            )


        self.get_logger().info(
            f"New position: x={self.x:.6f}, y={self.y:.6f}, theta={self.theta:.6f}"
        )

        # Calculate quaternion from yaw (theta)
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set the velocity
        if self.use_constant_velocity:
            odom_msg.twist.twist.linear.x = self.constant_vx
            odom_msg.twist.twist.linear.y = self.constant_vy
            odom_msg.twist.twist.angular.z = self.constant_vth
        else:
            # Calculate velocities from deltas
            linear_velocity = (delta_right + delta_left) / (2 * dt) if dt > 0 else 0
            angular_velocity = (
                (delta_right - delta_left) / (wheel_base * dt) if dt > 0 else 0
            )

            odom_msg.twist.twist.linear.x = linear_velocity
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Also publish the transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # Ensure motors are stopped and serial connection is closed before shutdown
        self.send_command("S")
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_controller = SerialMotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Node stopped by user")
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
