#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

#通过OpenCV读取图片信息
img = cv2.imread('save_image_20241217_141238.jpg')
# BGR图转为HSV
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 提取hsv中H通道数据
h = hsv[:, :, 0].ravel()
# 直方图显示
plt.hist(h, 180, [0, 180])
plt.show()
