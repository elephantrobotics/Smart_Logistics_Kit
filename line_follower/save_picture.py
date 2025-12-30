#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:
    def __init__(self):
        # Create cv_bridge and image publishers and subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)

    def callback(self, data):
        # Use cv_bridge to convert ROS image data into OpenCV image format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

        # Get current timestamp and format it
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        filename = f"/home/lanni/save_image_{timestamp}.jpg"

        # Save Image (Take a Photo)
        cv2.imwrite(filename, cv_image)
        print("Image saved!")

        # Unregister the subscriber to stop receiving images
        self.image_sub.unregister()

        # Display the captured image
        cv2.imshow("Captured Image", cv_image)
        cv2.waitKey(0)  # Wait for a keyboard event to ensure the window is displayed
        cv2.destroyAllWindows()  # Close Window

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()