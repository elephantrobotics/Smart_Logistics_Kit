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

    # pump control function
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
    for i in range(pick_times): #i=1,the delivery box picks up only one at a time, first recognize one layer
        mc.send_angles(angle_watch, 80) # Camera photo position
        wait()

        while True :
            qr_texts,tvecs =scanner.start_capture() # Get QR code city information and tvec displacement matrix
            time.sleep(1)
            print("qr_texts",qr_texts)
            print("tvecs",tvecs)

            if qr_texts is not None : 
                curr_coords = mc.get_coords() # Get current pose
                print("curr_coords",curr_coords)
                time.sleep(2)
                
                while curr_coords is None:
                    time.sleep(0.5)
                    curr_coords=mc.get_coords()
                    print("coords_s is None")
                    if curr_coords is not None:
                        break

                mat = homo_transform_matrix(*curr_coords) @ homo_transform_matrix(-10, -35, 10, 0, 0, 0)  

                # X error compensation
                p_base[0] -=10
                # Y error compensation
                p_base[1] +=40
                # Z-axis fixed height
                p_base[2] = box_height

                new_coords = np.concatenate([p_base, curr_coords[3:]]) # Concatenate x, y, z, and the current posture into a new array
                print("move_coords",list(new_coords))  # Pay attention to lines 152/153. Will an error occur if the robotic arm doesn't descend enough?。
                mc.send_coords(list(new_coords),20,1)
                wait()

                map_navigation.pump_on()
                time.sleep(2)
                print("pump_on")

                curr_coords = mc.get_coords() # Get current pose
                print("curr_coords",curr_coords)
                time.sleep(2)
                
                while curr_coords is None:
                    time.sleep(0.5)
                    curr_coords=mc.get_coords()
                    print("coords_s is None")
                    if curr_coords is not None:
                        break
                
                curr_coords[2]+=40  # Z-axis height compensation
                mc.send_coords(curr_coords,40,mode=1) # Z-axis height compensation
                wait()

                mc.send_angles(angle_table["pick_point2"], 50)
                wait()

                mc.send_angles(angle_table["place_init"], 80)
                wait()

                coords_s = mc.get_coords() # Get current pose
                print(coords_s)
                time.sleep(2)
                
                while coords_s is None:
                    time.sleep(0.5)
                    coords_s=mc.get_coords()
                    print("coords_s is None")
                    if coords_s is not None:
                        break

                hight= 45
                coords_s[2]-=hight
                mc.send_coords(coords_s,40,mode=1) #Z-axis descent
                wait()
                map_navigation.pump_off()
                time.sleep(2)
                print("pump_off")

                coords_s[2]+=hight        
                mc.send_coords(coords_s,40,mode=1) #Z-axis height compensation
                wait()

                mc.send_angles(angle_table["place_point4"], 50) # Transition point, prevent crashing into the box
                wait()

                scanner = None
                scanner = QRCodeScanner()
                break

            else:
                print("qr scanner failed")

    # Reset after completion
    mc.send_angles(angle_table["move_init"], 50)
    time.sleep(1)
    return qr_texts

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
            
            # Target point coordinates
            x_goal, y_goal, orientation_z, orientation_w = goal
            print(f"Navigate to target point: x={x_goal}, y={y_goal}, orientation_z={orientation_z}, orientation_w={orientation_w}")
            
            # Execute navigation
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
            # Perform OCR recognition based on whether the target is reached
            if flag_feed_goalReached:
                if i > 0:                    
                    recognized_ocr_texts.append(ocr_capture.start_capture())  # Recognize text and store box variables
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
    
    box_goals_0 = [
        [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919],#Point number one in the middle, facing forward
        [-1.1358978748321533,1.0654418468475342,0.8887916047179362,0.45831155711253446],#Middle point number two, facing forward
        [-1.7721543312072754,1.5437819957733154,0.8958855713120555,0.4442848670784004] #Middle point number three, facing forward
    ]
    box_goals_1 = [
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#Point number one in the middle, facing backward
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#Point number one in the middle, facing backward
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#Middle point number two, facing backward
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#Middle point number two, facing backward
        [-1.7721543312072754,1.5437819957733154,-0.4625007280446197,0.8866189015344736] #Middle point number three, facing backward
    ]

    box_goals_2 = [
        [-0.7391788959503174,0.2486436188220978,0.2948542038624959,0.9555422536259784],#Box number one, facing forward
        [-0.7391788959503174,0.2486436188220978,-0.953971689949811,0.29989667349655913],#Box number two, facing forward
        [-1.1358978748321533,1.0654418468475342,0.29911497291546424,0.9542170785401931],#Box number three, facing forward
        [-1.1358978748321533,1.0654418468475342,-0.952682813485717,0.30396621011049635],#Box number four, facing forward
        [-1.7721543312072754,1.5437819957733154,-0.9580734245756398,0.28652279686249366]#Box number five, facing forward
    ]
    
    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 15.2, -21.88, 0.96, 90.79, 4.57],    #Camera photo position 2, center point of the delivery box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #Pick transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 68.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place box point
    "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, 1.46]
    }
    
    city_to_region_mapping = {
        'Beijing': 'North China',
        'Shanghai': 'East China',
        'Nanjing': 'East China',
        'Dongguan': 'South China',
        'Guangzhou': 'South China',
        'Wuhan': 'North East',
        'Dalian': 'North East',
    }
    
    pack_pose = [-1.5043163299560547,2.357182264328003,0.2875093576208822,0.9577778287684612,0.06853892326654787]
    boxes_with_text = []
    recognized_ocr_texts = ['South China','North East','North China','East China'] #Fixed express sorting points
    recognized_ocr = [] # OCR recognition adds express sorting points
    recognized_qr_texts = []
    
    initialized = True # Navigation initial actions
    box_1_height = 60   #Express box layer 1 Z-axis height 60, demo2 90
    
    map_navigation = MapNavigation()
    ocr_capture = OCRVideoCapture()
    scanner = QRCodeScanner()
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)
    
    plist = get_port_list()
    print(plist)
    
    mc = MechArm270('/dev/ttyACM0',115200) # Connect to the robot arm
    mc.set_fresh_mode(0)
    mc.send_angles(angle_table["move_init"], 50)
    wait()
    
    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)
    
    for text, box_goals_1, box_goals_2 in zip(recognized_ocr_texts, box_goals_1, box_goals_2):
        box_info = {
            "text": text,
            "box_goals_1": box_goals_1,
            "box_goals_2": box_goals_2,
        }
        boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print(box)
    
    if (initialized):
        initialized = False
        xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
        map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) #amcl relocation
    
        angle_pick = angle_table["pick_watch"]
        box_height = box_1_height
               
        recognized_qr_texts.append(pick(angle_pick,box_height))  # Send the camera joint angle and Z-axis height to pick and return the recognized text
    
        # x shifts 1 second
        map_navigation.pub_vel(-0.1,0,0)
        time.sleep(2.5)
        map_navigation.pub_vel(0,0,0)

        # Rotate 180°
        map_navigation.pub_vel(0,0,-0.1)
        time.sleep(4)
        map_navigation.pub_vel(0,0,0)

        # x shifts 1 second
        map_navigation.pub_vel(0.1,0,0)
        time.sleep(2)
        map_navigation.pub_vel(0,0,0)
        
    else:
        print("failed.")
    
    if recognized_qr_texts: #recognized_qr_texts = ['Shanghai', 'Nanjing', 'Wuhan', 'Beijing', 'Dalian']  # City-level list obtained through OCR
        print("recognized_ocr_texts:",recognized_ocr_texts) #debug
        print("recognized_qr_texts:",recognized_qr_texts)   #debug 
        # Get the last city in the list
        last_city = recognized_qr_texts[-1]
        region = city_to_region_mapping.get(last_city, "Unknown area") # Shanghai: East China Region, Return to East China Region
        if region != "Unknown area":
            # Find the target position corresponding to that region
            for box in boxes_with_text:
                if box["text"] == region:

                    # Get the two target points for that region
                    box_goals_1 = box["box_goals_1"]
                    box_goals_2 = box["box_goals_2"]

                    # Traverse the target points and direction information, navigate to each target in turn
                    for target_num, goal in enumerate([box_goals_1, box_goals_2], 1):
                        # Target coordinates
                        x_goal, y_goal, orientation_z, orientation_w = goal

                        print(f"Navigate to target {region}in{target_num}: x={x_goal}, y={y_goal}, Direction z={orientation_z}, Direction w={orientation_w}")
                        map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                    recognized_ocr.append(ocr_capture.start_capture())

                    print(f"Recognized {recognized_ocr[-1]},{region} express will be transported to {recognized_ocr[-1]}")
