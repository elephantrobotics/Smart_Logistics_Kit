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
        self.processor = processor  # Pass in the CameraProcessor object
        self.running = True  # Control thread stopping
        # Initialize ROS node
        rospy.init_node('robot_control', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_aligned = False  # Record if the car has aligned with the Aruco

    def run(self):
        # Start the camera's various tasks
        while self.running:  # Control thread loop
            self.processor.show_camera()  # Call the show_camera method in CameraProcessor

    def stop(self):
        self.running = False  # Set stopping flag
        self.processor.stop_camera()
     
    def get_real_time_data(self):
        # Return real-time data from CameraProcessor
        return self.processor.pose_data[0], self.processor.pose_data[4], self.processor.pose_data[6][0]/960.0

    def fnShutDown():
        rospy.loginfo("Shutting down. cmd_vel will be 0")

    def control_y_translation(self, perc):
        """
        Control the car to move along the y-axis to align with the center of the Aruco.
        """
        cmd = Twist()
        error_y = perc - 0.5  # Assuming the center of the Aruco is at perc = 0.5
        cmd.linear.y = error_y * 0.5  # Control translation speed based on error

        # Publish control command
        self.cmd_vel_pub.publish(cmd)

        # Check if aligned, if error is less than threshold (e.g., 0.05), consider aligned
        if abs(error_y) < 0.05:
            self.is_aligned = True  # Set as aligned
            print("Aruco center aligned")

    def control_x_translation(self, z):
        """
        Control the car to move along the x-axis based on the distance to the Aruco.
        """
        cmd = Twist()
        if z > 0.1:  # Assuming z represents the distance to the Aruco, stop when less than 0.1
            cmd.linear.x = 0.1  # Control forward speed based on z value
            print("Moving forward...")
        else:
            cmd.linear.x = 0.0  # Stop when distance is less than threshold
            print("Stopped, reached target distance")

        # Publish control command
        self.cmd_vel_pub.publish(cmd)

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    real_time_data.stop()  # Ensure thread stops
    real_time_data.join()  # Wait for thread to exit
    sys.exit(0)  # Exit program

if __name__ == "__main__":
    # Set camera parameters (can be adjusted as needed)
    camera_matrix = np.array([[785.855437, 0.000000, 451.670922], 
                              [0.000000, 584.820336, 259.056856],
                              [0.000000, 0.000000, 1.000000]], dtype=np.float32)
    dist_matrix = np.array([0.095135, -0.109279, -0.002513, -0.002433, 0.000000], dtype=np.float32)

    # Create CameraProcessor instance
    camera_processor = CameraProcessor(camera_matrix, dist_matrix)

    # Start RealTimeData thread, passing in CameraProcessor instance
    real_time_data = RealTimeData(camera_processor)
    real_time_data.start()

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    # Real-time data acquisition
    while True:
        # Get real-time data z, ry, and pose_data_dict
        z, ry, perc = real_time_data.get_real_time_data()

        print(f"Real-time acquired z: {z}, ry: {ry}, percentage: {perc}")
