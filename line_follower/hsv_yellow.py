#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#Reading image information using OpenCV
img = cv2.imread('save_image_20241217_141238.jpg')
# BGR chart converted to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

cv2.imshow("img", img)

lower_yellow = np.array([10,43,46])
upper_yellow = np.array([25,255,255])

mask = cv2.inRange(hsv,lower_yellow,upper_yellow)

cv2.imshow("mask", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
