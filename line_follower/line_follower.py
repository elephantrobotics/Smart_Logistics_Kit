#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class image_converter:
    def __init__(self):
        self.publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

        hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([10,43,46])
        upper_yellow = np.array([25,255,250])

        mask = cv2.inRange(hsv,lower_yellow,upper_yellow)

        h,w,d = cv_image.shape

        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] =0
        mask[search_bot:h, 0:w] =0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image,(cx,cy),20,(0,0,255),-1)
            err = cx -w/2
            twist = Twist()
            twist.linear.x = 0.01
            twist.angular.z = -float(err) /500
            self.publisher.publish(twist)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()
        empty_message = Twist()
        image_converter.publisher.publish(empty_message)
