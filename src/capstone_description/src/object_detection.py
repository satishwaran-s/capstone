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

        # distance Smoothing (Moving Average Filter over last 5 readings)
        self.distance_buffer = deque(maxlen=5)

        # camera calibration parameter
        self.KNOWN_WIDTH = 15  # with of the object in cm
        self.FOCAL_LENGTH = 500  # adjusted bsed on calibration

    def estimate_distance(self, object_width_in_pixels):
        # estimate distance using object width in pixels and apply a moving average filter 
        if object_width_in_pixels > 0:
            raw_distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / object_width_in_pixels
            self.distance_buffer.append(raw_distance)
            return sum(self.distance_buffer) / len(self.distance_buffer)  # moving avergage filter
        return None

    def image_callback(self, msg):
        # convert the ROS2 image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # convert to HSV for color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define color range for colour detection 
        # lower_red1 = np.array([0, 120, 70])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([170, 120, 70])
        # upper_red2 = np.array([180, 255, 255])
        lowwer_brwon = np.array([10, 50, 50])
        upper_brwon = np.array([30, 150, 150])

        # create mask for objects (handling wrap-around in HSV)
        # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # mask = mask1 + mask2
        mask = cv2.inRange(hsv, lowwer_brwon, upper_brwon)

        # find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # process detected objects
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            if w > 20:  # ignore small detections (noise filtering)
                distance = self.estimate_distance(w)
                
                if distance:
                    self.get_logger().info(f"Object detected at {distance:.2f} cm")
                    
                    # draw a bounding box and distance text so we can see
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # bounding box will be a green box
                    cv2.putText(frame, f"{distance:.2f} cm", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # to test if changes when object is closer than 30 cm
                    if distance < 30 and not self.object_detected:
                        self.object_detected = True
                        self.get_logger().info("STOPPP")
                        # Here, publish velocity commands to stop the robot

                    elif distance >= 30 and self.object_detected:
                        self.object_detected = False
                        self.get_logger().info("MOVING ...")

        # show the processed frame
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
