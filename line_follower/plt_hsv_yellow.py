#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

img = cv2.imread('save_image_20241217_141238.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

h = hsv[:, :, 0].ravel()
plt.hist(h, 180, [0, 180])
plt.show()
