import threading
import time
import signal
import numpy as np
import rospy
import cv2
import sys
from cuda_simple_camera import CameraProcessor  

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class RealTimeData(threading.Thread):
    def __init__(self, processor):
        super().__init__()
        self.processor = processor  
        self.running = True  
        rospy.init_node('robot_control', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_aligned = False  

    def run(self):
        while self.running:
            self.processor.show_camera() 

    def stop(self):
        self.running = False 
        self.processor.stop_camera()
     
    def get_real_time_data(self):
        return self.processor.pose_data[0], self.processor.pose_data[4], self.processor.pose_data[6][0]/960.0

    def fnShutDown():
        rospy.loginfo("Shutting down. cmd_vel will be 0")

    def control_y_translation(self, perc):
        """
        Control the movement of the small car along the y-axis to align it with the center of the Aruco perc
        """
        cmd = Twist()
        error_y = perc - 0.5  
        cmd.linear.y = error_y * 0.5  

        # Publish the control command to the small car
        self.cmd_vel_pub.publish(cmd)

        if abs(error_y) < 0.05:
            self.is_aligned = True  # Set the alignment flag to True
            print("Aligned with Aruco center")

    def control_x_translation(self, z):
        """
        Control the movement of the small car along the x-axis to move forward or backward
        """
        cmd = Twist()
        if z > 0.1: 
            cmd.linear.x = 0.1  
            print("Moving forward...")
        else:
            cmd.linear.x = 0.0  # Stop moving forward
            print("Moving backward")

        # Publish the control command to the small car
        self.cmd_vel_pub.publish(cmd)

# class ControllerThread(threading.Thread):
#     def __init__(self, ros_controller, real_time_data):
#         super().__init__()
#         self.ros_controller = ros_controller
#         self.real_time_data = real_time_data

#     def run(self):
#         while not rospy.is_shutdown():
#             z, ry, perc = self.real_time_data.get_real_time_data()

#             if not self.ros_controller.is_aligned:
#                 self.ros_controller.control_y_translation(perc)
#             else:
#                 self.ros_controller.control_x_translation(z)

#             time.sleep(0.1) 

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    real_time_data.stop()  

    camera_processor = CameraProcessor(camera_matrix, dist_matrix)
    real_time_data = RealTimeData(camera_processor)
    real_time_data.start()

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    while True:
        z, ry, perc = real_time_data.get_real_time_data()

        print(f"Real-time data: z: {z}, ry: {ry}, Percent:{perc}")
