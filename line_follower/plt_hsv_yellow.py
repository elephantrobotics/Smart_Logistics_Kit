#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Use OpenCV to read image information
img = cv2.imread('save_image_20241217_141238.jpg')
# Convert BGR image to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Extract H channel data from HSV image
h = hsv[:, :, 0].ravel()
# Display histogram of H channel
plt.hist(h, 180, [0, 180])
plt.show()
