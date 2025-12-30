import rospy
import time
import actionlib
import signal
import sys
import os
import numpy as np
import Jetson.GPIO as GPIO

from pymycobot.mecharm270 import MechArm270
from pymycobot.utils import get_port_list
from OCRVideoCapture import OCRVideoCapture
from QRCodeScanner import QRCodeScanner
from Transformation import homo_transform_matrix
from wit_usb2can import SerialCANParser

from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class MapNavigation:
    def __init__(self):
        self.goalReached = False
        rospy.init_node('map_navigation', anonymous=False)
        
        # ros publisher
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pub_setpose = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(19, GPIO.OUT)
        GPIO.setup(26, GPIO.OUT)
        self.pump_off()

    # init robot  pose AMCL
    def set_pose(self, xGoal, yGoal, orientation_z, orientation_w,covariance):
        pose = PoseWithCovarianceStamped()
        pose.header.seq = 0
        pose.header.stamp.secs = 0
        pose.header.stamp.nsecs = 0
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = xGoal
        pose.pose.pose.position.y = yGoal
        pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, 1.57)  
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = orientation_z
        pose.pose.pose.orientation.w = orientation_w
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0,0.0, 0.0, 0.0, covariance]
        rospy.sleep(1)
        self.pub_setpose.publish(pose)
        rospy.loginfo('Published robot pose: %s' % pose)
    
    # move_base
    def moveToGoal(self, xGoal, yGoal, orientation_z, orientation_w):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      
            sys.exit(0)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position =  Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = orientation_z 
        goal.target_pose.pose.orientation.w = orientation_w

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal) 

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
        
    # speed command
    def pub_vel(self, x, y , theta):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = theta
        self.pub.publish(twist)

    # Suction Pump Control Function
    def pump_on(self):
        GPIO.output(26, GPIO.LOW)
        GPIO.output(19, GPIO.HIGH)

    def pump_off(self):
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(19, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(19, GPIO.HIGH)
       
def pick(angle_watch,box_height,pick_times=1):
    global scanner 
    mc.send_angles(angle_watch, 80) # Camera pose
    wait()

    while True :
        qr_texts,tvecs =scanner.start_capture() # Get QR code city information and tvec displacement matrix
        time.sleep(1)
        print("qr_texts",qr_texts)
        
        if qr_texts is not None : 
            curr_coords = mc.get_coords() # Get current pose
            print("curr_coords",curr_coords)
            time.sleep(2)
        else:
            print("qr scanner failed")
            
def load():
    mc.send_angles([0,0,0,0,0,0], 60)
    wait()

    mc.send_angles(angle_table["place_init"], 50)
    wait()

    coords_s = mc.get_coords() # Get current pose
    print(coords_s)
    wait()
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]-=70
    mc.send_coords(coords_s,40,mode=1) # z-axis lowering
    wait()
    map_navigation.pump_on()
    wait()
    print("pump_off")

    coords_s = mc.get_coords()
    print(coords_s)
    wait()
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]+=70
    mc.send_coords(coords_s,40,mode=1) # z-axis lifting
    wait()

    mc.send_angles(angle_table["place_point4"], 50)
    wait()

    mc.send_angles(angle_table["place_point2"], 50)
    wait()

    mc.send_angles(angle_table["place_point3"], 50)
    wait()
    
    map_navigation.pump_off()
    time.sleep(2)

    # Reset after completion
    mc.send_angles(angle_table["move_init"], 50)
    wait()


def ocr_recognized():
    # List of target points, organized according to the order you provided
    goals_sequence = [
        (box_goals_0[0], box_goals_2[0], box_goals_2[1]),  # box_goals_0 1 point -> box_goals_2 1 point, 2 point
        (box_goals_0[1], box_goals_2[2], box_goals_2[3]),  # box_goals_0 2 point -> box_goals_2 3 point, 4 point
        (box_goals_0[2], box_goals_2[4])                   # box_goals_0 3 point -> box_goals_2 5 point
    ]

    # Navigate by traversing the sequence of target points
    for goal_set in goals_sequence:
        for i, goal in enumerate(goal_set):
            
            # Target coordinates
            x_goal, y_goal, orientation_z, orientation_w = goal
            print(f"Navigate to the target point: x={x_goal}, y={y_goal}, Direction z={orientation_z}, Direction w={orientation_w}")
            
            # Execute navigation
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
            # Execute OCR recognition based on whether the target is reached
            if flag_feed_goalReached:
                if i > 0:                    
                    recognized_ocr_texts.append(ocr_capture.start_capture())  # Recognize text and store box variable
            else:
                recognized_ocr_texts.append(None)  # If the target is not reached, append None

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    # Close all connections
    running_flag = False
    print("Connections closed.")
    sys.exit()

def wait():
    time.sleep(0.3)
    state = mc.is_moving()
    while(state != 0):
        state = mc.is_moving()
        time.sleep(0.1)
        
if __name__ == '__main__':
    print("python OCRVideoCapture")
    os.system('python OCRVideoCapture.py')

    print("python agv_aruco")
    os.system('python agv_aruco.py')    # Navigation target point
    
    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 15.2, -21.88, 0.96, 90.79, 4.57],    #Camera photo position 2, center point of the delivery box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #Capture transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 68.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place box position
    "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, 1.46]
    }
    map_navigation = MapNavigation()
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)
    
    mc = MechArm270('/dev/ttyACM0',115200) # Connect to the mecharm
    mc.set_fresh_mode(0)
    mc.send_angles(angle_table["move_init"], 50)
    wait()
                        
    load()  #Put the box down
    map_navigation.pub_vel(-0.1,0,0)
    time.sleep(5.7)
    map_navigation.pub_vel(0,0,0)
    
