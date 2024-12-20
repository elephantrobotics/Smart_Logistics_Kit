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
        # 创建cv_bridge和图像的订阅者
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)

    def callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)

        # 获取当前时间戳并格式化
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        filename = f"/home/lanni/save_image_{timestamp}.jpg"

        # 保存图像（拍摄一张照片）
        cv2.imwrite(filename, cv_image)
        print("Image saved!")

        # 停止订阅，只拍摄一张图像
        self.image_sub.unregister()

        # 显示拍摄的图像
        cv2.imshow("Captured Image", cv_image)
        cv2.waitKey(0)  # 等待键盘事件，确保窗口显示

        cv2.destroyAllWindows()  # 关闭窗口

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()