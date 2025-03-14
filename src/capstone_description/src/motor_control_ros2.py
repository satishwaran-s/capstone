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

        # parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("wheel_base", 0.33)
        self.declare_parameter("drift_threshold", 0.5)
        self.declare_parameter("linear_scale", 1.0) 
        self.declare_parameter("angular_scale", 1.0)

        # get parameters
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
        self.linear_scale = self.get_parameter("linear_scale").get_parameter_value().double_value
        self.angular_scale = self.get_parameter("angular_scale").get_parameter_value().double_value


        # initialise serial connection to arduno
        try:
            self.serial_port = serial.Serial(self.serial_port_name, 115200, timeout=1)
            self.get_logger().info(
                f"Connected to Arduino via Serial on {self.serial_port_name}"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e

        self.running = True
        self.odom_timer = self.create_timer(0.05, self.read_encoder_and_publish_odom)

        # ROS2 publishers and subscribers
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

        # odom variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.prev_linear = 0.0
        self.prev_angular = 0.0

        # start keyboard listener
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def update_odometry(self, linear_distance, angular_rotation):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.001:
            return

        self.theta += angular_rotation
        self.x += linear_distance * math.cos(self.theta)
        self.y += linear_distance * math.sin(self.theta)
        self.last_time = now

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        self.odom_publisher.publish(odom_msg)
        self.broadcast_transform()

    def broadcast_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def send_command(self, command):
        try:
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def print_final_odometry(self):
        self.get_logger().info(
            f"Final Odometry - Total X: {self.x:.2f}, Total Angular: {self.theta:.2f}"
        )

    def keyboard_listener(self):
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
                self.reset_odometry()  # reset odometry to be used when neeeded 
            elif key in ["q", "Q"]:
                self.get_logger().info("Shutting down...")
                self.running = False
                self.send_command("S")  # stop motors before exit
                
                # clearing the serial buffer so that it tries to print the final odom values instead of waiting
                while self.serial_port.in_waiting > 0:
                    self.serial_port.readline()
                
                # print final odom  
                self.print_final_odometry()
                break

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        if abs(linear_speed) < 0.05 and abs(angular_speed) < 0.05:
            self.send_command("S")
        elif abs(angular_speed) > 0.5:
            if angular_speed > 0:
                self.send_command("L")  # turn left
            else:
                self.send_command("R")  # turn right
        else:
            if linear_speed > 0:
                self.send_command("F")  # forward
            else:
                self.send_command("B")  # backward

        self.get_logger().info(
            f"Received cmd_vel - Linear: {linear_speed}, Angular: {angular_speed}"
        )

    def reset_odometry(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.get_logger().info("Odometry reset.")

    def apply_drift_correction(self):
        drift_distance = math.sqrt(self.x * self.x + self.y * self.y)
        if drift_distance > self.DRIFT_THRESHOLD:
            excess_drift = drift_distance - self.DRIFT_THRESHOLD
            max_excess = 1.0
            scale = min(excess_drift / max_excess, 1.0)
            correction_factor = 1.0 - (0.05 * scale)
            self.x *= correction_factor
            self.y *= correction_factor
            self.get_logger().info(
                f"Drift correction applied: {correction_factor:.4f}, Position: ({self.x:.2f}, {self.y:.2f})"
            )

    def read_encoder_and_publish_odom(self):    
        if self.serial_port.in_waiting > 0:
            try:
                # read and decode data
                data = self.serial_port.readline().decode().strip()
                
                self.get_logger().debug(f"Raw serial data: {data}")

                # initialise values
                linear_distance = None
                angular_rotation = None

                # split data into components (handles "D:... A:... T:..." format)
                parts = data.split()
                for part in parts:
                    if part.startswith('D:'):
                        linear_distance = float(part[2:])  # extracts value after D:
                    elif part.startswith('A:'):
                        angular_rotation = float(part[2:])  # extract value after A:
                    # T: timestamp is purposely ignored cos it may affect how data is parsed

                # validate parsed values
                if linear_distance is not None and angular_rotation is not None:
                    self.publish_odometry_from_distance_angle(
                        linear_distance * self.linear_scale,
                        angular_rotation * self.angular_scale
                    )
                else:
                    # warning about missing components which are the values it skipped (NOT TO BE ALARMED)
                    missing = []
                    if linear_distance is None: missing.append("linear")
                    if angular_rotation is None: missing.append("angular")
                    self.get_logger().warn(
                        f"Partial data: {data} | Missing: {', '.join(missing)}"
                    )

            except UnicodeDecodeError as e:
                self.get_logger().error(f"Decode error: {e} | Raw bytes: {self.serial_port.readline()}")
                self.serial_port.reset_input_buffer()
            except ValueError as e:
                self.get_logger().error(f"Value error: {e} | Bad data: {data}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                self.serial_port.reset_input_buffer()

    def publish_odometry_from_distance_angle(self, linear_distance, angular_rotation):
        # use ROS time for dt calculation
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # dt in seconds
        if dt <= 0.0:
            self.get_logger().warn("Invalid dt, skipping update")
            return
        self.last_time = now



        # update position using current orientation, linear and angular scale added as scale factors but currently set to 1 as raw data received is good
        delta_linear = (linear_distance - self.prev_linear) * self.linear_scale
        delta_angular = (angular_rotation - self.prev_angular) * self.angular_scale

        # delta_linear = linear_distance
        # delta_angular = angular_rotation

        delta_x = delta_linear * math.cos(self.theta)
        delta_y = delta_linear * math.sin(self.theta)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_angular

        # normalise theta to -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # save current values for next iteration
        self.prev_linear = linear_distance
        self.prev_angular = angular_rotation

        # create and publish odometry message using ROS time
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # orientation
        qz = math.sin(self.theta * 0.5)
        qw = math.cos(self.theta * 0.5)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # velocity
        odom_msg.twist.twist.linear.x = delta_linear / dt
        odom_msg.twist.twist.angular.z = delta_angular / dt

        # pose covariance 
        odom_msg.pose.covariance = [
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
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(
            f"Published Odometry: ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})"
        )

        # publish transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
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
        self.send_command("S")
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        motor_controller = SerialMotorController()
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Node stopped by user")
    except Exception as e:
        motor_controller.get_logger().error(f"Error: {e}")
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
