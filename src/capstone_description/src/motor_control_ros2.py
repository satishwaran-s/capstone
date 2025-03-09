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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SerialMotorController(Node):
    def __init__(self):
        super().__init__("serial_motor_controller")

        # Parameters
        self.declare_parameter("use_constant_velocity", False)
        self.declare_parameter("constant_vx", 0.2)
        self.declare_parameter("constant_vy", 0.0)
        self.declare_parameter("constant_vth", 0.1)
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_base", 0.33)
        self.declare_parameter("drift_threshold", 0.5)
        self.declare_parameter("scale_factor", 2.0)

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
        self.serial_port_name = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.wheel_base = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        self.DRIFT_THRESHOLD = (
            self.get_parameter("drift_threshold").get_parameter_value().double_value
        )
        self.scale_factor = (
            self.get_parameter("scale_factor").get_parameter_value().double_value
        )

        # Initialize serial connection to Arduino
        try:
            self.serial_port = serial.Serial(self.serial_port_name, 115200, timeout=1)
            self.get_logger().info(
                f"Connected to Arduino via Serial on {self.serial_port_name}"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e

        self.running = True

        # Create a timer for odometry updates which reads every 0.1 seconds
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
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform from base_link to laser if using a lidar
        self.publish_static_transforms()

        # Start the keyboard listener thread for manual control
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        # Odometry Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.prev_linear = 0.0
        self.prev_angular = 0.0

    def publish_static_transforms(self):
        # Publish static transform from base_link to laser
        # Adjust these values based on your robot's physical configuration
        laser_transform = TransformStamped()
        laser_transform.header.stamp = self.get_clock().now().to_msg()
        laser_transform.header.frame_id = self.base_frame
        laser_transform.child_frame_id = "laser"

        # Set the position of the laser relative to base_link
        # Adjust these values based on your robot's physical dimensions
        laser_transform.transform.translation.x = 0.1  # 10cm forward from base_link
        laser_transform.transform.translation.y = 0.0  # centered on y-axis
        laser_transform.transform.translation.z = 0.15  # 15cm above base_link

        # No rotation (laser is aligned with base_link)
        laser_transform.transform.rotation.x = 0.0
        laser_transform.transform.rotation.y = 0.0
        laser_transform.transform.rotation.z = 0.0
        laser_transform.transform.rotation.w = 1.0

        # self.static_tf_broadcaster.sendTransform(laser_transform)
        # self.get_logger().info("Published static transform from base_link to laser")

    def send_command(self, command):
        # Send command to Arduino
        try:
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

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
            "Use W/A/S/D for movement, P to start pump, O to stop pump, Q to quit, R to reset odometry"
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
            elif key in ["r", "R"]:
                self.reset_odometry()  # reset odometry
            elif key in ["q", "Q"]:
                self.get_logger().info("Shutting down...")
                self.running = False
                self.send_command("S")  # stop motors before exit
                break

    def cmd_vel_callback(self, msg):
        # Callback to handle cmd_vel messages
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Map linear and angular speeds to motor commands
        if abs(linear_speed) < 0.05 and abs(angular_speed) < 0.05:
            # If speeds are very small, stop motors
            self.send_command("S")
        elif abs(angular_speed) > 0.5:
            # If angular speed is high, turn in place
            if angular_speed > 0:
                self.send_command("L")  # Turn left
            else:
                self.send_command("R")  # Turn right
        else:
            # Forward/backward movement
            if linear_speed > 0:
                self.send_command("F")  # Forward
            else:
                self.send_command("B")  # Backward

        self.get_logger().info(
            f"Received cmd_vel - Linear: {linear_speed}, Angular: {angular_speed}"
        )

    def read_encoder_and_publish_odom(self):
        if self.serial_port.in_waiting > 0:
            try:
                data = self.serial_port.readline().decode().strip()
                self.get_logger().debug(f"Received serial data: {data}")

                # Changed pattern to match the Arduino output format D:x.xxxx A:x.xxxx
                linear_pattern = r"D:(-?[\d.]+)"
                angular_pattern = r"A:(-?[\d.]+)"

                linear_match = re.search(linear_pattern, data)
                angular_match = re.search(angular_pattern, data)

                if linear_match and angular_match:
                    linear_distance = float(linear_match.group(1))
                    angular_rotation = float(angular_match.group(1))
                    self.publish_odometry_from_distance_angle(
                        linear_distance, angular_rotation
                    )
                    self.get_logger().debug(
                        f"Processed data - Linear: {linear_distance:.4f}, Angular: {angular_rotation:.4f}"
                    )
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
        self.prev_linear = 0.0
        self.prev_angular = 0.0
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

    def publish_odometry_from_distance_angle(self, linear_distance, angular_rotation):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt < 0.001:
            self.get_logger().warn("Time delta too small, skipping odometry update")
            return

        # Calculate change in linear and angular since last update
        delta_linear = linear_distance - self.prev_linear
        delta_angular = angular_rotation - self.prev_angular

        # Save current values for next iteration
        self.prev_linear = linear_distance
        self.prev_angular = angular_rotation
        self.last_time = current_time

        # Update robot position and orientation
        delta_theta = delta_angular
        avg_theta = self.theta + delta_theta / 2.0

        delta_x = delta_linear * math.cos(avg_theta)
        delta_y = delta_linear * math.sin(avg_theta)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to prevent drift over time
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Apply drift correction if enabled
        self.apply_drift_correction()

        # Calculate quaternion from yaw (theta)
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Set the velocity
        linear_velocity = delta_linear / dt if dt > 0 else 0
        angular_velocity = delta_angular / dt if dt > 0 else 0

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # Set covariance - important for mapping
        # Using diagonal matrix with low uncertainty for position, higher for orientation
        pose_cov = [
            0.05,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.05,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.5,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.5,
        ]
        twist_cov = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.3,
        ]

        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Also publish the transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
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

    try:
        motor_controller = SerialMotorController()
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        if "motor_controller" in locals():
            motor_controller.get_logger().info("Node stopped by user")
    except Exception as e:
        if "motor_controller" in locals():
            motor_controller.get_logger().error(f"Error: {e}")
        else:
            print(f"Error during initialization: {e}")
    finally:
        if "motor_controller" in locals():
            motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

