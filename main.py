#!/usr/bin/env python
#coding=UTF-8
import os
import timer
import rospy
from std_srvs.srv import SetBool
import time
import actionlib
import signal
import sys
import numpy as np
import xmlrpc
import Jetson.GPIO as GPIO
import glob
import numpy as np
import socket
import subprocess
from pymycobot.mecharm270 import MechArm270
from pymycobot.utils import get_port_list

from OCRVideoCapture import OCRVideoCapture
from QRCodeScanner import QRCodeScanner
from Transformation import homo_transform_matrix
from wit_usb2can import SerialCANParser
from MapNavigation import MapNavigation
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

def detect_devices():
    device_status={
        'active_arm':'/dev/ttyACM0',
        'dctive_camera':'/dev/video1',
        'arm_found':False,
        'camera_found':False
    }

    if os.path.exists('/dev'):
        serial_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        device_status['arm_available'] = serial_ports

        if '/dev/ttyACM0' in serial_ports:
            device_status['arm_found'] = True
        elif serial_ports:
            device_status['active_arm'] = serial_ports[0]  
    
    if os.path.exists('/dev'):
        camera_ports = glob.glob('/dev/video*')
        device_status['camera_available'] = camera_ports

        if '/dev/video1' in camera_ports:
            device_status['camera_found'] = True
            detected_camera = 'video1'
        elif '/dev/video2' in camera_ports:
            device_status['active_camera'] = '/dev/video2'
            device_status['camera_found'] = True
            detected_camera = 'video2'
    
    if detected_camera:
        print(f"Camera detected as availabel: {detected_camera}")
        
    else:
        print("No available camera detected")

    return device_status    


def get_rpc_proxy(url="http://localhost:6666/", timeout=20):
    proxy = xmlrpc.client.ServerProxy(url)
    start_time = time.time()
    
    print("Waiting for ArUco RPC Server to wake up...")
    while time.time() - start_time < timeout:
        try:
            # We call a dummy system method to check if the server is alive
            proxy.system.listMethods() 
            print("Connected to RPC Server successfully!")
            return proxy
        except (ConnectionRefusedError, OSError):
            time.sleep(1) # Wait 1 second before retrying
            
    raise Exception(f"Could not connect to RPC server after {timeout} seconds.")
def pick(angle_watch, box_height, pick_info=None, pick_times=1):
    """The function for capturing parcel boxes supports handling stacking scenarios.
    
    Parameters:
    angle_watch: Angle of the camera's capture position
    box_height: Grab height
    pick_info: A dictionary containing the target ID and whether it's a parent layer
    pick_times: Number of times to grab the target
    """
    global scanner 
    for i in range(pick_times): #i=1, only one package is picked up at a time.
        scanner = None
        scanner = QRCodeScanner(device_status['dctive_camera'])
        

        retry_count = 0
        max_retries = 3
        success = False
        last_valid_qr = None
        back_basket_qr = None
        while retry_count < max_retries:
            try:
                mc.send_angles(angle_watch, 80) # Camera position
                wait(angle_watch)
                qr_texts, tvecs = scanner.start_capture() # Obtain QR code city information and tvec displacement matrix
                time.sleep(1)
                print("Package information identified：", qr_texts,"Calculated tvecs:", tvecs)
                if qr_texts !=last_valid_qr and last_valid_qr is not None :
                    print("Confirmation: The old result is different from the current result. The data retrieval was successful!")
                    return last_valid_qr  # Return to previously recorded QR code information
                if qr_texts is not None and tvecs is not None: 
                    last_valid_qr = qr_texts
                    curr_coords = mc.get_coords()
                    time.sleep(1)
                    
                    while curr_coords is None:
                        print("获取坐标失败，重试中... (coords is None)")
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                    print("获取成功:", curr_coords)
                    # Matrix transformation calculation to retrieve coordinates
                    mat = homo_transform_matrix(*curr_coords) @ homo_transform_matrix(-10, -35, 10, 0, 0, 0)  # Hand-eye matrix
                    p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1]) # Convert to homogeneous coordinates
                    p_base = np.squeeze((mat @ p_end)[:-1]).astype(int) # Calculate the base coordinates

                    new_coords = np.concatenate([[p_base[0]-46,p_base[1]+30,box_height], curr_coords[3:]]) # Combined into complete coordinates
  
                    coords = list(new_coords)
                    print("Move to the grab position, coordinates are：", list(coords))
                    mc.send_coords(coords, 60, 1)

                    time.sleep(0.2) 

                    start_wait = time.time()
                    is_started = False
                    while time.time() - start_wait < 1.0:
                        if mc.is_moving():
                            is_started = True
                            break
                        time.sleep(0.05) # Fast polling

                    if not is_started:
                        print("Warning: The robot did not start moving within 1 second after the command was sent (this may be due to communication delay or the command being discarded).")

                    start_move_time = time.time()
                    while mc.is_moving(): 
                        if time.time() - start_move_time > 15:
                            print("Warning: Movement timeout will force exit.")
                            break
                        time.sleep(0.1)

                    check_coords = mc.get_coords()
                    if check_coords is None:
                        print("Warning: Failed to obtain coordinates after moving.")
                    else:
                        target_z = coords[2]
                        current_z = check_coords[2]

                        if abs(current_z - target_z) > 10:
                            print(f"Critical Error: Target Altitude Not Reached! Target Z={target_z}, Current Z={current_z}")
                            continue
                        else:
                            print(f"Position confirmed successful; Z-axis error: {abs(current_z - target_z):.2f}")
                    print("pump_on")
                    map_navigation.pump_on()
                    time.sleep(2)

                    curr_coords = mc.get_coords()
                    while curr_coords is None:
                        print("Failed to obtain coordinates, retrying.... (coords is None)")
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                    print("Current coordinates：", curr_coords)

                    lift_height = 60 if (pick_info and pick_info.get("is_upper", False)) else 50
                    print(f"Lifting height: {lift_height} {'(upper box)' if (pick_info and pick_info.get('is_upper', False)) else '(lower box)'}")
                    curr_coords[2] += lift_height  # z-axis raised

                    mc.send_coords(curr_coords,40, mode=1) #z-axis raised
                    wait(curr_coords, 1)

                    mc.send_angles(angle_table["pick_point2"], 50)
                    wait(angle_table["pick_point2"], 0)

                    mc.send_angles(angle_table["place_init"], 80)
                    wait(angle_table["place_init"], 0)

                    curr_coords = mc.get_coords() #Get current pose
                    while curr_coords is None:
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                    print(curr_coords)
                    hight = 45
                    curr_coords[2] -= hight
                    mc.send_coords(curr_coords, 40, mode=1) #z-axis decrease
                    wait(coords, 1) # Use precise coordinates to check
                    map_navigation.pump_off()
                    time.sleep(2)
                    print("pump_off")

                    curr_coords[2] += hight        
                    mc.send_coords(curr_coords, 40, mode=1) #z-axis raised
                    wait(coords, 1) # Use precise coordinates to check

                    mc.send_angles(angle_table["place_point4"], 50) # Transition point to prevent the box from being knocked over.
                    wait(angle_table["place_point4"], 0) # Use precise angle position check
                    back_basket_qr, _ = scanner.start_capture()
                    if back_basket_qr is not None:
                        print("Confirmation: Package detected in the backpack. Successful capture!")
                        return back_basket_qr  # Return to previously recorded QR code information
                    retry_count += 1
                    print(f"A capture attempt has been completed, and the system is returning to the photo capture location for confirmation.... (frequency{retry_count}/{max_retries})")
                    
                else:
                    print("QR code not detected. Retrying.")
                    retry_count += 1
                    time.sleep(1)
                    if retry_count >= max_retries:
                        print(f"Reaching the maximum number of retries({max_retries})，Fetch failed")
                        # Try initializing a new scanner
                        scanner = None
                        scanner = QRCodeScanner(device_status['dctive_camera'])
            except Exception as e:
                print(f"Error during crawling: {str(e)}，Retrying...")
                retry_count += 1
                time.sleep(1)
                scanner = None
                scanner = QRCodeScanner(device_status['dctive_camera'])

    # Reset after completion

    mc.send_angles(angle_table["move_init"], 50)
    wait(angle_table["move_init"], 0) # Use precise angle position check
    return last_valid_qr

def load():

    angles = angle_table["place_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use precise angle position check

    coords_s = mc.get_coords() #Get current pose
    print(coords_s)
    wait()
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]-=70
    coords = coords_s
    speed = 40
    mc.send_coords(coords, speed, mode=1) #z-axis decrease
    wait(coords, 1) # Use precise coordinates to check
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
    coords = coords_s
    speed = 40
    mc.send_coords(coords, speed, mode=1) #z-axis raised
    wait(coords, 1) # Use precise coordinates to check

    angles = angle_table["place_point4"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use precise coordinates to check

    angles = angle_table["place_point2"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use precise coordinates to check

    angles = angle_table["place_point3"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use precise coordinates to check
    
    map_navigation.pump_off()
    time.sleep(2)

    # Reset after completion
    angles = angle_table["move_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use precise angle position check

def ocr_recognized():  # The visual recognition error is too large, so this function is not in use at this time.
        
    # The list of target points, organized in the order you specify.
    goals_sequence = [
        (box_goals_0[0], box_goals_2[0], box_goals_2[1]),  # box_goals_0 Point 1 -> box_goals_2 Point 1, 2
        (box_goals_0[1], box_goals_2[2], box_goals_2[3]),  # box_goals_0 Point 2 -> box_goals_2 Point 3, 4
        (box_goals_0[2], box_goals_2[4])                   # box_goals_0 Point 3 -> box_goals_2 Point 5
    ]

    # Navigate by traversing the target points in sequence
    for goal_set in goals_sequence:
        for i, goal in enumerate(goal_set):
            
            #Target coordinates
            x_goal, y_goal, orientation_z, orientation_w = goal
            print(f"Navigate to target point: x={x_goal}, y={y_goal}, direction z={orientation_z}, direction w={orientation_w}")
            
            # Execute navigation
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
            # Perform OCR recognition based on whether the target is reached
            if flag_feed_goalReached:
                if i > 0:                    
                    recognized_ocr_texts.append(ocr_capture.start_capture())  # Recognize text and store the box's variables
            else:
                recognized_ocr_texts.append(None)  # If the target is not reached, append None

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    # Close all connections
    running_flag = False
    print("Connections closed.")
    sys.exit()

def wait(data=None, ids=0):

    import traceback
    import time
    
    # If no data parameter is provided, the original simple wait logic will be used.
    if data is None:
        time.sleep(0.3)
        state = mc.is_moving()
        while(state != 0):
            state = mc.is_moving()
            time.sleep(0.1)
        return
    
    # Otherwise, use the precise check logic of check_position.
    try:
        start_time = time.time()
        while True:
            # Timeout Detection
            if (time.time() - start_time) >= 4.5:
                print("The wait function timed out.")
                break
            res = mc.is_in_position(data, ids)
            if res == 1:
                break
            time.sleep(0.1)
    except Exception as e:
        e = traceback.format_exc()
        print(e)

if __name__ == '__main__':

    # Define the target positions for id3 and id4
    box_goals_0 = [
        [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919],#Middle Point 1, pose facing forward
        [-1.1358978748321533,1.0654418468475342,0.8887916047179362,0.45831155711253446],#Middle Point 2, pose facing forward    
        [-1.7721543312072754,1.5437819957733154,0.8958855713120555,0.4442848670784004] #Middle Point 3, pose facing forward
    ]

    box_goals_1 = [
        [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766],#Pose of Box 1
        [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218],#Pose of Box 2
        [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766],
        [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218]
    ]
    
    # Define the target positions for id3 and id4
    box_goal_id3 = [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766]  # Target point position of ID3
    box_goal_id4 = [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218] # Target point position of ID4

    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#Middle Point 1, pose facing forward
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#Middle Point 1, pose facing backward
    
    pack_goal = [0.4767872333526611,0.04532311737537384,0.06401975195944533,0.9979486316234173]#Near the parcel sorting bin
    charge_goal =[-0.5688837170600891,-0.31650811433792114,0.4588518939959322,0.8885127682686084]

    pack_pose = [0.8529961109161377,0.050533026456832886,0.0112260354743728,0.9999369860783869,0.06853892326654787]

    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 10.2, -21.88, 0.96, 90.79, 0.0],   #Camera photo position 2, center point of the delivery box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],   #Pick transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 75.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place box position
    "place_point4":[-93.36, 20.03, -22.85, -3.07, 89, 1.46]
    }

    city_to_region_mapping = {
        'Beijing ': 'North China',
        'Shanghai': 'East China',
        'Nanjing': 'East China',
        'Dongguan': 'South China',
        'Guangzhou': 'South China',
        'Wuhan': 'North East',
        'Dalian': 'North East',
    }

    # initialized = True # Initial navigation action
    USB_CAN_Enable = False # Whether to enable communication with the charging device
    box_2_height = 58  #Height of the second layer of the delivery box 101, demo2 140
    box_1_height = 14   #Height of the first layer of the delivery box 60, demo2 90

    boxes_with_text = []
    recognized_ocr_texts = ['South China','North East','North China','East China'] #Fixed delivery sorting points
    recognized_ocr = [] # OCR-recognized courier sorting point
    recognized_qr_texts = []
    PICK_TIMES = 999 # Loop pick times

    device_status = detect_devices()

    print("===== Device inspection results ====")
    print(f"Robotic arm status: {'Found' if device_status['arm_found'] else 'Default port not found,using' + device_status['active_arm']}")
    print(f"Camera status: {'Found' if device_status['camera_found'] else 'Default port not found,using'}")
    if 'arm_available' in device_status and device_status['arm_available']:
        print(f"Available serial ports: {device_status['arm_available']}")
    if 'camera_avaivle' in device_status and device_status['camera_avaible']:
        print(f"Avaible cameras: {device_status['camera_avaible']}")
    print("====================================")

    map_navigation = MapNavigation()
    ocr_capture = OCRVideoCapture()
    scanner = QRCodeScanner(device_status['dctive_camera'])
    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)
    socket.setdefaulttimeout(300)
    proc = subprocess.Popen(['python3', 'agv_aruco_1.py'])
    proxy = get_rpc_proxy()
    print(get_port_list())
    # mc = MechArm270('/dev/ttyACM0',115200) # Connect the robotic arm
    mc = MechArm270('/dev/ttyACM0',115200) # Connect the robotic arm and enable debug mode
    mc.set_fresh_mode(0)

    mc.send_angles(angle_table["move_init"], 50)
    wait()

    # Register the Ctrl+C signal handler
    global running_flag
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    ##########################################################
    # # Function 1: Record information for three navigation points
    ##########################################################
    # ocr_recognized() # Visual recognition of delivery sorting points, error rate is high, this function is not used for now

    # Correctly initialize the boxes_with_text list to ensure data structure matches
    for i, text in enumerate(recognized_ocr_texts):
        if i < len(box_goals_1):
            box_info = {
                "text": text,
                "box_goals_1": box_goals_1[i]
            }
            boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print("boxes with text",box)

    ##########################################################
    # # Function 2: Loop pick PICK_TIMES times, each time only pick one box, then sort the box
    ##########################################################
    timer=timer.TaskTimer()
    x_goal, y_goal, orientation_z, orientation_w = pack_goal #Navigate to the express sorting shelf
    flag_feed_goalReached = False
    while not flag_feed_goalReached:
        print("Trying to reach pack_goal...")
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if not flag_feed_goalReached:
            print("Navigation failed, retrying...")
        time.sleep(2)  # Add a delay to prevent frequent calls
    for i in range(PICK_TIMES):
        timer.reset()
        # Initialize the target_box variable
        target_box = None
        # if (initialized):
        #     initialized = False
        #     x_goal, y_goal, orientation_z, orientation_w = goal_1
        #     map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            

        # After reaching pack_goal, detect QR codes on id3 and id4
        target_info = proxy.aruco_rpc("check_box_qrcodes")
        target_box = target_info["target_id"]
        is_upper = target_info["is_upper"]

        # When target_box is None, it means that there is no delivery box on id3 and id4
        if target_box is None:
            print("The car is at the pack_goal position, start continuously detecting whether there is a new delivery box put in...")
            # Continuously detect until a new delivery box is detected
            while target_box is None:
                print("Wait for a while to detect a new delivery box...")
                # Call the check_box_qrcodes() function again to detect
                target_info =  proxy.aruco_rpc("check_box_qrcodes")
                target_box = target_info["target_id"]
                is_upper = target_info["is_upper"]
                # If still not detected, wait for a while and detect again
                if target_box is None:
                    print("Still not detected a delivery box, wait for 3 seconds and detect again...")
                    time.sleep(3)

        print(f"Detected a new delivery box, select the target: {target_box}, whether the higher level: {is_upper}")
        timer.lap("Determine the number of items and the upper/lower layers.")
        print("python agv_aruco")
        proxy.aruco_rpc("align")
        timer.lap("Approaching the package")
        # xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
        # map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) 

        
        angle_pick = angle_table["pick_watch"]
        # Dynamically set the capture height based on whether it's an upper-level box # The function for determining the number of boxes is not working well (it occasionally fails). Currently, there's only one box, so it's temporarily commented out.
        # if is_upper:
        #     box_height = box_2_height
        #     print(f"Set the grab height of the upper box: {box_height}")
        # else:
        #     box_height = box_1_height
        #     print(f"Set the grab height of the lower box: {box_height}")
        box_height = box_1_height
        recognized_qr_texts.append(pick(angle_pick, box_height, target_info))  # Send the camera joint angle, Z-axis height, and target information to capture and return the recognized text.
        timer.lap("Fetch complete")
        # Translate x for 1 second
        map_navigation.pub_vel(-0.1,0,0)
        time.sleep(4.5)
        map_navigation.pub_vel(0,0,0)

        # Turn right 180°
        map_navigation.pub_vel(0,0,-0.1)
        time.sleep(4)
        map_navigation.pub_vel(0,0,0)

        # Translate x for 1 second
        map_navigation.pub_vel(0.1,0,0)
        time.sleep(2)

    ##########################################################
    # # Function 3: Loop navigate to each city in the recognized_qr_texts list
    ##########################################################
        if recognized_qr_texts:#recognized_qr_texts = ['Shanghai', 'Nanjing', 'Wuhan', 'Beijing', 'Dalian'] # List of cities obtained through OCR
            print("recognized_ocr_texts:",recognized_ocr_texts) #debug
            print("recognized_qr_texts:",recognized_qr_texts)   #debug 
            # Get the last city in the list
            last_city = recognized_qr_texts[-1]
            region = city_to_region_mapping.get(last_city, "Unknown area") # Map the last city to the corresponding region, default to "Unknown area" if not found
            print(f"City: {last_city}, Corresponding region: {region}")
            if region != "Unknown area":
                # Find the target location corresponding to this area
                region_found = False
                for box in boxes_with_text:
                    print(f"Check the box: {box['text']}")
                    if box["text"] == region:
                        region_found = True
                        # Loop navigate to each target point and direction
                        box_goals_1 = box["box_goals_1"]

                        # Iterate through the target points and direction information, and navigate to each target in turn.
                        for target_num, goal in enumerate([box_goals_1], 1):
                            # Target coordinates
                            x_goal, y_goal, orientation_z, orientation_w = goal

                            print(f"Navigate to the target {target_num} in {region}: x={x_goal}, y={y_goal}, direction z={orientation_z}, direction w={orientation_w}")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        recognized_ocr.append(ocr_capture.start_capture())

                        print(f"Recognized the text: {recognized_ocr[-1]}, {region} express will be delivered to {recognized_ocr[-1]}")

                        print("python agv_aruco")
                        proxy.aruco_rpc("unload")
                        timer.lap("Near the basket")
                        load()  #After navigating to all points, place the box.
                        timer.lap("discharge")
                        timer.save_to_txt()
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(3.7)
                        map_navigation.pub_vel(0,0,0)
        x_goal, y_goal, orientation_z, orientation_w = pack_goal #Navigate to the express sorting shelf
        flag_feed_goalReached = False
        while not flag_feed_goalReached:
            print("Trying to reach pack_goal...")
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        # time.sleep(1)
        # map_navigation.pub_vel(0,0.15,0)
        # time.sleep(1)
        # map_navigation.pub_vel(0,0,0)
        timer.lap("Return to the starting point")

    sys.exit()  #End program
