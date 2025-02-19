import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf2_ros import TransformBroadcaster, TransformStamped
import math

class FakeOdometryPublisher(Node):
    def __init__(self):
        super().__init__('fake_odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.get_logger().info("Fake odometry publisher started")

    def publish_odometry(self):
        # Fake movement (modify this to simulate real movement)
        self.x += 0.05  # Move forward slowly
        self.theta += 0.01  # Slight rotation

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom_msg)

        # Publish TF transform (odom → base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose, Twist, TransformStamped
# from tf2_ros import TransformBroadcaster
# import math

# class FakeOdometryPublisher(Node):
#     def __init__(self):
#         super().__init__('fake_odometry_publisher')
#         self.publisher = self.create_publisher(Odometry, '/odom', 10)
#         self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         self.get_logger().info("Fake odometry publisher started")

#     def publish_odometry(self):
#         # Update position and orientation (simulate motion)
#         self.x += 0.01  # Simulate moving forward
#         self.theta += 0.001  # Simulate rotating (radians)

#         # Create and publish the odometry message
#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = 'odom'
#         odom_msg.child_frame_id = 'base_link'

#         # Set the pose (position)
#         odom_msg.pose.pose.position.x = self.x
#         odom_msg.pose.pose.position.y = self.y
#         odom_msg.pose.pose.position.z = 0.0

#         # Compute quaternion from yaw (theta)
#         qz = math.sin(self.theta / 2.0)
#         qw = math.cos(self.theta / 2.0)
#         odom_msg.pose.pose.orientation.x = 0.0
#         odom_msg.pose.pose.orientation.y = 0.0
#         odom_msg.pose.pose.orientation.z = qz
#         odom_msg.pose.pose.orientation.w = qw

#         # Set the twist (velocity)
#         odom_msg.twist.twist.linear.x = 0.01  # Fake forward speed
#         odom_msg.twist.twist.angular.z = 0.001  # Fake rotational speed

#         self.publisher.publish(odom_msg)

#         # Broadcast the dynamic transform from odom to base_link
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.translation.z = 0.0
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = qz
#         t.transform.rotation.w = qw
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FakeOdometryPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
