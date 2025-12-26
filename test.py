#!/usr/bin/env python
#coding=UTF-8
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
    for i in range(pick_times): #i=1,the delivery box picks up only one at a time, first recognizing one layer
        mc.send_angles(angle_watch, 80) # Camera shooting position
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
                p_base = np.squeeze((mat @ p_end)[:-1]).astype(int) 

                # X error compensation
                p_base[0] -=10
                # Y error compensation
                p_base[1] +=40
                # Z-axis fixed height
                p_base[2] = box_height

                new_coords = np.concatenate([p_base, curr_coords[3:]]) #Concatenate x, y, z, and the current posture into a new array
                print("move_coords",list(new_coords))
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
                
                curr_coords[2]+=40  # Z-axis lift
                mc.send_coords(curr_coords,40,mode=1) #Z-axis lift
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
                mc.send_coords(coords_s,40,mode=1) #Z-axis lift
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
    mc.send_coords(coords_s,40,mode=1) #Z-axis descent
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
    mc.send_coords(coords_s,40,mode=1) #Z-axis lift
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
            
            # Navigate to the target point
            x_goal, y_goal, orientation_z, orientation_w = goal
            print(f"Navigate to the target point: x={x_goal}, y={y_goal}, Direction_z={orientation_z}, Direction_w={orientation_w}")
            
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
        [-1.1358978748321533,1.0654418468475342,0.8887916047179362,0.45831155711253446],#Point number two in the middle, facing forward      
        [-1.7721543312072754,1.5437819957733154,0.8958855713120555,0.4442848670784004] #Point number three in the middle, facing forward
    ]

    box_goals_1 = [
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#Point number one in the middle, facing backward
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#Point number one in the middle, facing backward
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#Point number two in the middle, facing backward
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#Point number two in the middle, facing backward
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#Point number two in the middle, facing backward
        [-1.7721543312072754,1.5437819957733154,-0.4625007280446197,0.8866189015344736] #Point number three in the middle, facing backward
    ]

    box_goals_2 = [
        [-0.7391788959503174,0.2486436188220978,0.2948542038624959,0.9555422536259784],#Box 1 posture
        [-0.7391788959503174,0.2486436188220978,-0.953971689949811,0.29989667349655913],#Box 2 posture
        [-1.1358978748321533,1.0654418468475342,0.29911497291546424,0.9542170785401931],#3Box 3 posture
        [-1.1358978748321533,1.0654418468475342,-0.952682813485717,0.30396621011049635],#Box 4 posture
        [-1.7721543312072754,1.5437819957733154,-0.9580734245756398,0.28652279686249366]#Box 5 posture
    ]

    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#Point number one in the middle, facing forward
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#Point number one in the middle, facing backward
    pack_goal = [-1.6927944374084473,1.6765453577041626,0.29911497291546424,0.9542170785401931]#Near the courier sorting box
    charge_goal =[-0.5688837170600891,-0.31650811433792114,0.4588518939959322,0.8885127682686084]

    pack_pose = [-1.5043163299560547,2.357182264328003,0.2875093576208822,0.9577778287684612,0.06853892326654787]

    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 15.2, -21.88, 0.96, 90.79, 4.57],    #Camera photo position 2, center point of the delivery box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #Pick transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 68.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place box position
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

    initialized = True # Navigation initial action
    USB_CAN_Enable = False # Whether to enable communication for the charging device
    box_2_height = 101  #Height of the second layer of the delivery box Z-axis 101, demo2 140
    box_1_height = 60   #Height of the first layer of the delivery box Z-axis 60, demo2 90

    boxes_with_text = []
    recognized_ocr_texts = ['East China','South China','North China','North East','North East'] #Fixed delivery points
    recognized_ocr = [] # OCR recognition adds express delivery sorting points
    recognized_qr_texts = []

    map_navigation = MapNavigation()
    ocr_capture = OCRVideoCapture()
    scanner = QRCodeScanner()
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)

    plist = get_port_list()
    print(plist)
    mc = MechArm270('/dev/ttyACM0',115200) # Connect to the mechatronic arm
    # mc = MechArm270('/dev/ttyACM0',115200,debug=1) # Connect to the mechatronic arm and enable debug mode
    mc.set_fresh_mode(0)

    mc.send_angles(angle_table["move_init"], 50)
    wait()

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    for i in range(5):
        # Capture based on the number of cycles, fixed camera shooting position, and aspiration height
        if i == 0:
            angle_pick = angle_table["pick_watch"]
            box_height = box_2_height
        elif i == 1:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height
        elif i == 2:
            angle_pick = angle_table["pick_watch"]
            box_height = box_2_height
        elif i == 3:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height
        elif i == 4:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height

        recognized_qr_texts.append(pick(angle_pick,box_height))  # Send the camera joint angles and Z-axis height, grab and return the recognized text

        load()  # Navigate to all points and perform box placement

    exit()

    ##########################################################
    # # Function 1: Record information for five navigation points
    ##########################################################
    # ocr_recognized() # Visual recognition of express delivery points, error rate is about 10%, not used for the time being

    for text, box_goals_1, box_goals_2 in zip(recognized_ocr_texts, box_goals_1, box_goals_2):
        box_info = {
            "text": text,
            "box_goals_1": box_goals_1,
            "box_goals_2": box_goals_2,
        }
        boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print(box)
        
    ##########################################################
    # # Function 2: Loop 5 times, each time only grabbing one box and then sorting it
    ##########################################################
    for i in range(5):
        if (initialized):
            initialized = False
            x_goal, y_goal, orientation_z, orientation_w = goal_1
            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
        x_goal, y_goal, orientation_z, orientation_w = pack_goal
        flag_feed_goalReached = False
        while not flag_feed_goalReached:
            print("Trying to reach pack_goal...")
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            if not flag_feed_goalReached:
                print("Navigation failed, retrying...")
                time.sleep(2)  # Add a small delay to prevent frequent calls

        print("python agv_aruco")
        os.system('python agv_aruco.py') 

        xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
        map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) #amcl重定位

        # Capture based on the number of cycles, fixed camera shooting position, and suction height
        if i == 0:
            angle_pick = angle_table["pick_watch"]
            box_height = box_2_height
        elif i == 1:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height
        elif i == 2:
            angle_pick = angle_table["pick_watch"]
            box_height = box_2_height
        elif i == 3:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height
        elif i == 4:
            angle_pick = angle_table["pick_watch"]
            box_height = box_1_height
    
        recognized_qr_texts.append(pick(angle_pick,box_height))  # Send the camera joint angles and Z-axis height, grab and return the recognized text

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

    ##########################################################
    # # Function 3: Traverse all recognized city names in order, navigate to each city
    ##########################################################
        if recognized_qr_texts: #recognized_qr_texts = ['Shanghai', 'Nanjing', 'Wuhan', 'Beijing', 'Dalian'] # List of cities obtained through OCR
            print("recognized_ocr_texts:",recognized_ocr_texts) #debug
            print("recognized_qr_texts:",recognized_qr_texts)   #debug 
            # Get the last city in the list
            last_city = recognized_qr_texts[-1]
            region = city_to_region_mapping.get(last_city, "Unknown area") # Map the last city to the corresponding region, default to "Unknown area" if not found
            print(f"City: {last_city}, Corresponding region: {region}")
            if region != "Unknown area":
                # Find the target location corresponding to this area
                for box in boxes_with_text:
                    if box["text"] == region:

                        # Get the two target points for this area
                        box_goals_1 = box["box_goals_1"]
                        box_goals_2 = box["box_goals_2"]

                        # Traverse the target points and direction information, navigate to each target in order
                        for target_num, goal in enumerate([box_goals_1, box_goals_2], 1):
                            # Target coordinates
                            x_goal, y_goal, orientation_z, orientation_w = goal

                            print(f"Navigate to {region} target {target_num}: x={x_goal}, y={y_goal}, Direction z={orientation_z}, Direction w={orientation_w}")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        recognized_ocr.append(ocr_capture.start_capture())

                        print(f"Recognized {recognized_ocr[-1]},{region} express will be transported to {recognized_ocr[-1]}")

                        print("python agv_aruco")
                        os.system('python agv_aruco.py')    # Navigate to the target point
                        
                        load()  # Navigate to all points and put the box
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(5.7)
                        map_navigation.pub_vel(0,0,0)
      
    ##########################################################
    # # Function 4: Return to the charging station
    ##########################################################

    x_goal, y_goal, orientation_z, orientation_w = goal_1_back # Navigate to this point first to avoid hitting the express placement box
    flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
    if flag_feed_goalReached:

        # x translate 1 second
        map_navigation.pub_vel(0.1,0,0)
        time.sleep(5.5)
        map_navigation.pub_vel(0,0,0)

        # Rotate 180°  
        map_navigation.pub_vel(0,0,0.1)
        time.sleep(4.2)
        map_navigation.pub_vel(0,0,0)

        if USB_CAN_Enable:
            parser.open_serial() # Open the USB serial port
            try:
                # Send AT commands to enter AT command mode from transparent transmission mode
                parser.send_at_commands(["AT+CG", "AT+AT"])
                
                while True:
                    # Start reading data
                    x_speed,z_speed,which_mode,infrared_bits = parser.read_serial_data()
                    if infrared_bits[7] == 0 :
                        if(which_mode) == 0x01:
                            map_navigation.pub_vel(x_speed,0,z_speed)
                        # elif (which_mode) == 0xBB: # 测压区
                        #     map_navigation.pub_vel(0,0,0)
                        #     time.sleep(1)
                        #     map_navigation.pub_vel(0,0,0.5)
                        #     time.sleep(1.15)
                        #     map_navigation.pub_vel(0,0,0)
                        #     time.sleep(0.5)
                        #     map_navigation.pub_vel(-0.1,0,0)
                        #     time.sleep(1)
                        #     map_navigation.pub_vel(0,0,0)
                        elif (which_mode) == 0xAA: # 充电区
                            map_navigation.pub_vel(0,0,0)
                            break
                        elif (which_mode) == 0xCF:
                            map_navigation.pub_vel(0,0,0)
                            break
                    else:
                        map_navigation.pub_vel(0,0,0)
                        break 
            except KeyboardInterrupt:
                print("Manually terminate the program.")
        else:
            # x translate 1 second
            map_navigation.pub_vel(-0.1,0,0)
            time.sleep(7.5)
            map_navigation.pub_vel(0,0,0)

        sys.exit()  # End the program 

        
