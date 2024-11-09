#!/usr/bin/env python3

# *******************************************************************************
# Script Name  : aruco_detector.py
# Author       : Daniel Sotelo Aguirre
# Date         : 08/11/2024
# Version      : v1.0
# *******************************************************************************

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('publish_image', True)
        self.publish_image = self.get_parameter('publish_image').value

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image, 
            '/processed_image',
            10
        )
        self.offset_publisher = self.create_publisher(
            Int32MultiArray,
            '/aruco_offset',
            10
        )
        
        self.bridge = CvBridge()

        # Camera parameters based on simulated camera
        # Calibration should be made in case of real camera
        self.width = 1280
        self.height = 720

        # Calculate the center of the camera frame
        self.cx = self.width // 2
        self.cy = self.height // 2

        # Create ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        
        try:
            # Convert ROS image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco marker
            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                # self.get_logger().info(f"Detected {len(ids)} ArUco markers.")
                # Draw markers and display ID
                # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):

                    # Calculate the 2D center position of the ArUco
                    c = corners[i][0]
                    center_x = int((c[:, 0].sum()) / 4)
                    center_y = int((c[:, 1].sum()) / 4)

                    # Calculate the offset from the center of the camera frame
                    offset_x = center_x - self.cx
                    offset_y = center_y - self.cy

                    # Publish the offset
                    offset_msg = Int32MultiArray()
                    offset_msg.data = [offset_x, offset_y]
                    self.offset_publisher.publish(offset_msg)

                    # Optionally draw the center and line on the image for visualization
                    if self.publish_image:
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.line(frame, (self.cx, self.cy), (center_x, center_y), (255, 0, 0), 2)
                        processed_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                        self.publisher.publish(processed_image_msg)

                    break

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args = None):
    rclpy.init(args = args)
    node = ArUcoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)