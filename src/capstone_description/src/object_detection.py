#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info("Object detection node has been started")

        self.object_detected = False

        # Distance Smoothing (Moving Average over last 5 readings)
        self.distance_buffer = deque(maxlen=5)

        # Camera Calibration Parameters
        self.KNOWN_WIDTH = 15  # cm (Adjust based on object size)
        self.FOCAL_LENGTH = 500  # Adjust based on calibration

    def estimate_distance(self, object_width_in_pixels):
        """Estimate distance using object width in pixels and apply a moving average filter."""
        if object_width_in_pixels > 0:
            raw_distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / object_width_in_pixels
            self.distance_buffer.append(raw_distance)
            return sum(self.distance_buffer) / len(self.distance_buffer)  # Moving average
        return None

    def image_callback(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to HSV for color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color range for red detection
        # lower_red1 = np.array([0, 120, 70])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([170, 120, 70])
        # upper_red2 = np.array([180, 255, 255])
        lowwer_brwon = np.array([10, 50, 50])
        upper_brwon = np.array([30, 150, 150])

        # Create mask for red objects (handling red wrap-around in HSV)
        # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # mask = mask1 + mask2
        mask = cv2.inRange(hsv, lowwer_brwon, upper_brwon)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process detected objects
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            if w > 20:  # Ignore small detections (noise filtering)
                distance = self.estimate_distance(w)
                
                if distance:
                    self.get_logger().info(f"Object detected at {distance:.2f} cm")
                    
                    # Draw bounding box and distance text
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box
                    cv2.putText(frame, f"{distance:.2f} cm", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Example: Stop robot if object is closer than 30 cm
                    if distance < 30 and not self.object_detected:
                        self.object_detected = True
                        self.get_logger().info("🚨 Object too close! Stopping robot...")
                        # Here, publish velocity commands to stop the robot

                    elif distance >= 30 and self.object_detected:
                        self.object_detected = False
                        self.get_logger().info("✅ Safe distance. Continuing movement...")
                        # Here, publish velocity commands to resume movement

        # Show the processed frame
        cv2.imshow("Obstacle Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


# import cv2
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class object_detection(Node):
#     def __init__(self):
#         super().__init__("object_detection")

#         self.subscription = self.create_subscription(
#             Image,
#             "/image_raw",
#             self.image_callback,
#             10
#         )
#         self.subscription

#         self.bridge = CvBridge()
#         self.get_logger().info("Object detection node has been started")

#         self.object_detected = False

#     def image_callback(self, msg):
#         # Convert the ROS2 Image message to an OpenCV image
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # Convert to HSV for color filtering
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#         # Define color range for red detection
#         lower_red = np.array([0, 120, 70])
#         upper_red = np.array([10, 255, 255])

#         # Create mask to detect red objects
#         mask = cv2.inRange(hsv, lower_red, upper_red)

#         # Find contours in the mask
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         # Check if red object is detected (if there are contours)
#         if len(contours) > 0 and not self.object_detected:
#             self.object_detected = True
#             self.get_logger().info("Object detected! Stopping robot...")
#             # Here you can stop the robot, e.g., by setting velocity to 0
#             # You would need to add motor control code here to stop movement
#             # e.g., publisher to stop robot velocity
#         elif len(contours) == 0 and self.object_detected:
#             self.object_detected = False
#             self.get_logger().info("Path cleared! Continuing movement...")
#             # Here you can resume movement, e.g., by publishing a movement command
#             # e.g., publisher to set robot velocity to move forward

#         # Draw bounding boxes around detected red objects
#         for contour in contours:
#             x, y, w, h = cv2.boundingRect(contour)
#             cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box

#         # Show the processed frame with bounding boxes
#         cv2.imshow("Obstacle Detection", frame)
#         cv2.waitKey(1)  # Refresh the image every 1ms
#     # def image_callback(self, msg):
#     #     # Convert ROS2 Image to OpenCV format
#     #     frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

#     #     # Convert to HSV for color filtering
#     #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     #     # Define color range for obstacle detection (example: red objects)
#     #     lower_color = np.array([0, 120, 70])   # Lower bound for red
#     #     upper_color = np.array([10, 255, 255]) # Upper bound for red

#     #     # Create mask to detect red objects
#     #     mask = cv2.inRange(hsv, lower_color, upper_color)

#     #     # Find contours in the mask
#     #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     #     # If any contours are found, an obstacle is detected
#     #     if len(contours) > 0:
#     #         self.get_logger().info("🚨 Obstacle Detected! Stopping Robot...")

#     #     # Draw bounding boxes around detected obstacles
#     #     for contour in contours:
#     #         x, y, w, h = cv2.boundingRect(contour)
#     #         cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green box

#     #     # Show the processed video (optional)
#     #     cv2.imshow("Obstacle Detection", frame)
#     #     cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = object_detection()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()