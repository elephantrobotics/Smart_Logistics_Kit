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
import http.client
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
    detected_camera = None
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
        print(f"Camera detected as available: {detected_camera}")
    else:
        print("No available camera detected")

    return device_status    

class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout=300, use_datetime=0):
        self.timeout = timeout
        super().__init__(use_datetime)

    def make_connection(self, host):
        # This forces the timeout on the underlying HTTP connection
        conn = http.client.HTTPConnection(host, timeout=self.timeout)
        return conn
def get_rpc_proxy(url="http://localhost:6666/", timeout=30):
    proxy = xmlrpc.client.ServerProxy(url, transport=TimeoutTransport(timeout=300))
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
import time
import socket
import xmlrpc.client

def safe_aruco_command(proxy, command, timeout_recovery=True):
    """
    Wraps the RPC call with timeout handling and status polling.
    Returns the server response OR 'Success (Recovered)'
    """
    print(f"--- Sending Command: {command} ---")
    
    try:
        # 1. Attempt the blocking call
        response = proxy.aruco_rpc(command)
        return response

    except socket.timeout:
        if not timeout_recovery:
            raise # Re-raise if we don't want to recover
            
        print(f"(!) Warning: Command '{command}' timed out (Network layer).")
        print("    The robot is likely still working. Switching to status monitoring...")

        # 2. Recovery Loop: Poll until robot is IDLE
        while True:
            try:
                # You MUST have 'get_robot_state' on the server for this to work
                status = proxy.get_robot_state()
                print(f"    -> Robot Status: {status}")
                
                if status == "IDLE" or status == "DONE":
                    print("    -> Robot finished task.")
                    return "Success (Recovered)"
                
                if "ERROR" in status:
                     return f"Robot Error: {status}"

            except Exception as e:
                print(f"    -> Polling failed ({e}), retrying...")
            
            time.sleep(2) # Don't flood the network
            
    except Exception as e:
        print(f"CRITICAL ERROR: RPC Failed completely: {e}")
        return None
def pick(angle_watch, box_height, pick_info=None, pick_times=1):
    """
    Function for picking up express boxes, supports handling stacked situations
    Parameters:
    angle_watch: Angle of the camera shooting position
    box_height: Picking height
    pick_info: Dictionary containing target ID and whether it is the upper layer
    pick_times: Number of times to pick up
    """
    global scanner 
    for i in range(pick_times): 
        scanner = None
        scanner = QRCodeScanner(device_status['dctive_camera'])
        

        retry_count = 0
        max_retries = 3
        success = False
        last_valid_qr = None
        back_basket_qr = None
        while retry_count < max_retries:
            try:
                mc.send_angles(angle_watch, 80) 
                wait(angle_watch)
                qr_texts, tvecs = scanner.start_capture() 
                time.sleep(1)
                print("Detected package information:", qr_texts,"Calculated tvecs:", tvecs)
                if qr_texts !=last_valid_qr and last_valid_qr is not None :
                    print("Confirmation: The old result is different from the current result. It is determined to be a successful capture!")
                    return last_valid_qr 
                if qr_texts is not None and tvecs is not None: 
                    last_valid_qr = qr_texts
                    curr_coords = mc.get_coords()
                    time.sleep(1)
                    
                    while curr_coords is None:
                        print("Failed to get coordinates, retrying... (coords is None)")
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                    print("Coordinates successfully retrieved:", curr_coords)
                    mat = homo_transform_matrix(*curr_coords) @ homo_transform_matrix(-10, -35, 10, 0, 0, 0)  
                    p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1]) 
                    p_base = np.squeeze((mat @ p_end)[:-1]).astype(int) 

                    new_coords = np.concatenate([[p_base[0]-46,p_base[1]+30,box_height], curr_coords[3:]]) 
  
                    coords = list(new_coords)
                    print("Moving to pick position, coordinates:", list(coords))
                    mc.send_coords(coords, 60, 1)

                    time.sleep(0.2) 

                    start_wait = time.time()
                    is_started = False
                    while time.time() - start_wait < 1.0:
                        if mc.is_moving():
                            is_started = True
                            break
                        time.sleep(0.05) 

                    if not is_started:
                        print("Warning: Command sent after 1 second, robot did not start moving (possible communication delay or command discarded)")

                    start_move_time = time.time()
                    while mc.is_moving(): 
                        if time.time() - start_move_time > 15:
                            print("Warning: Moving timeout, forced exit")
                            break
                        time.sleep(0.1)

                    check_coords = mc.get_coords()
                    if check_coords is None:
                        print("Warning: Failed to get coordinates after moving")
                    else:
                        target_z = coords[2]
                        current_z = check_coords[2]

                        if abs(current_z - target_z) > 10:
                            print(f"Critical Error: Robot did not reach target height! Target Z={target_z}, Current Z={current_z}")
                            continue
                        else:
                            print(f"Position confirmation successful, Z-axis error: {abs(current_z - target_z):.2f}")
                    print("pump_on")
                    map_navigation.pump_on()
                    time.sleep(2)

                    curr_coords = mc.get_coords()
                    while curr_coords is None:
                        print("Failed to get coordinates, retrying... (coords is None)")
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                    print("Current coordinates:", curr_coords)

                    lift_height = 60 if (pick_info and pick_info.get("is_upper", False)) else 50
                    print(f"Lift height: {lift_height} {'(Upper box)' if (pick_info and pick_info.get('is_upper', False)) else '(Lower box)'}")
                    curr_coords[2] += lift_height  

                    mc.send_coords(curr_coords,40, mode=1) 
                    wait(curr_coords, 1)

                    mc.send_angles(angle_table["pick_point2"], 50)
                    wait(angle_table["pick_point2"], 0)

                    mc.send_angles(angle_table["place_init"], 80)
                    wait(angle_table["place_init"], 0)

                    curr_coords = mc.get_coords() 
                    while curr_coords is None:
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                    print(curr_coords)
                    hight = 45
                    curr_coords[2] -= hight
                    mc.send_coords(curr_coords, 40, mode=1) 
                    wait(coords, 1) 
                    map_navigation.pump_off()
                    time.sleep(2)
                    print("pump_off")

                    curr_coords[2] += hight        
                    mc.send_coords(curr_coords, 40, mode=1) 
                    wait(coords, 1) 

                    mc.send_angles(angle_table["place_point4"], 50) 
                    wait(angle_table["place_point4"], 0) 
                    back_basket_qr, _ = scanner.start_capture()
                    if back_basket_qr is not None:
                        print("Confirmation: A package has been detected in the back basket, determined as successfully grabbed！")
                        return back_basket_qr 
                    retry_count += 1
                    print(f"Completed one grab attempt, returning to photo position for confirmation... (attempt {retry_count}/{max_retries})")
                    
                else:
                    print("No package QR code detected, retrying...")
                    retry_count += 1
                    time.sleep(1)
                    if retry_count >= max_retries:
                        print(f"Reached maximum retry attempts ({max_retries}, grab failed.")
                        scanner = None
                        scanner = QRCodeScanner(device_status['dctive_camera'])
            except Exception as e:
                print(f"Error during grab: {str(e)}, retrying...")
                retry_count += 1
                time.sleep(1)
                scanner = None
                scanner = QRCodeScanner(device_status['dctive_camera'])

    mc.send_angles(angle_table["move_init"], 50)
    wait(angle_table["move_init"], 0) 
    return last_valid_qr

def load():

    angles = angle_table["place_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) 
    coords_s = mc.get_coords() 
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
    mc.send_coords(coords, speed, mode=1) 
    wait(coords, 1) 
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
    mc.send_coords(coords, speed, mode=1)
    wait(coords, 1) 

    angles = angle_table["place_point4"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) 

    angles = angle_table["place_point2"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0)

    angles = angle_table["place_point3"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) 

    map_navigation.pump_off()
    time.sleep(2)

    angles = angle_table["move_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) 

shutdown_in_progress = False

def signal_handler(signal, frame):
    global running_flag, shutdown_in_progress
    if shutdown_in_progress:
        return
    shutdown_in_progress = True
    
    print("\nCtrl+C pressed. Exiting...")
    running_flag = False
    
    try:
        print("Stopping AGV movement...")
        if 'map_navigation' in globals() and map_navigation is not None:
            map_navigation.cancel_goal()
            time.sleep(0.2)
            map_navigation.pub_vel(0, 0, 0)
            time.sleep(0.5)
        print("AGV stopped.")
    except Exception as e:
        print(f"Error stopping AGV: {e}")
    
    try:
        print("Stopping AGV ArUco subprocess...")
        if 'proc' in globals() and proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except:
                proc.kill()
            print("AGV ArUco subprocess terminated.")
    except Exception as e:
        print(f"Error terminating subprocess: {e}")
    
    try:
        print("Releasing camera resources...")
        if 'scanner' in globals() and scanner is not None:
            scanner.release_resources()
        if 'ocr_capture' in globals() and ocr_capture is not None:
            ocr_capture.release_resources()
        print("Camera resources released.")
    except Exception as e:
        print(f"Error releasing camera resources: {e}")
    
    try:
        print("Turning off pump...")
        if 'map_navigation' in globals() and map_navigation is not None:
            map_navigation.pump_off()
            GPIO.cleanup()
        print("Pump turned off and GPIO cleaned up.")
    except Exception as e:
        print(f"Error cleaning up GPIO: {e}")
    
    try:
        print("Closing serial ports...")
        if 'parser' in globals() and parser is not None:
            parser.close()
        print("Serial ports closed.")
    except Exception as e:
        print(f"Error closing serial ports: {e}")
    
    print("All resources cleaned up. Exiting...")
    os._exit(0)

def wait(data=None, ids=0):
    """
    Enhanced wait function, compatible with the original simple calls, while also supporting precise position checking
    :param data: Angle or coordinate data, default is None (only checks if movement has stopped)
    :param ids: Angle-0, Coordinate-1, default is 0
    :param max_same_data_count: Maximum threshold count for consecutive identical data
    """
    import traceback
    import time

    if data is None:
        time.sleep(0.3)
        state = mc.is_moving()
        while(state != 0):
            state = mc.is_moving()
            time.sleep(0.1)
        return
    
    try:
        start_time = time.time()
        while True:
            if (time.time() - start_time) >= 4:
                print("wait function timeout")
                break
            res = mc.is_in_position(data, ids)
            if res == 1:
                break
            time.sleep(0.1)
    except Exception as e:
        e = traceback.format_exc()
        print(e)

if __name__ == '__main__':


    box_goals_1 = [
        [0.08485770225524902,-0.20438778042793274,-0.7170147446397785,0.6970580004340766],#Pose of Box 1
        [0.7079846858978271,-0.2074418902397156,-0.6723159317868799,0.7402643364809218],#Pose of Box 2
        [0.4793493449687958, -0.35094438314437866, -0.700840908443522, 0.7133176158290632],
        [0.0007295608520507812,-0.35475303888320923, -0.700286274565093,0.7138621251024201]
    ]
    box_pose_1 = [
        [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766,0.25],#Pose of Box 1
        [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218,0.25],#Pose of Box 2
        [0.4793493449687958, -0.65094438314437866, -0.700840908443522, 0.7133176158290632,0.10],
        [0.0007295608520507812,-0.65475303888320923, -0.700286274565093,0.7138621251024201,0.10]
    ]

    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#Middle point number one, facing forward
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#Middle point number one, facing backward
    
    pack_goal = [0.4053445506095886,0.031757590174675,0.0022835285580446907,0.9999973927452634]#Pack goal
    charge_goal =[-0.5688837170600891,-0.31650811433792114,0.4588518939959322,0.8885127682686084]

    pack_pose = [0.7311730257294759,-0.08994807983106717,-0.030164998620566538,0.9995449328860715,0.060571319298727874]

    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 10.2, -21.88, 0.96, 90.79, 0.0],    #Camera position 2, center of the box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #Pick transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 75.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place point
    "place_point4":[-93.36, 20.03, -22.85, -3.07, 89, 1.46]
    }

    city_to_region_mapping = {
        # Chinese Keys
        '北京市': '华北区',
        '上海市': '华东区',
        '南京市': '华东区',
        '东莞市': '华南区',
        '广州市': '华南区',
        '武汉市': '华东区',
        '大连市': '东北区',
        
        # English Keys (Lowercase)
        'Beijing City': 'North China',
        'Shanghai City': 'East China',
        'Nanjing City': 'East China',
        'Dongguan City': 'South China',
        'Guangzhou City': 'South China',
        'Wuhan City': 'North East',
        'Dalian City': 'North East',
    }

    # initialized = True 
    USB_CAN_Enable = False 
    box_2_height = 58 
    box_1_height = 14   

    boxes_with_text = []
    recognized_ocr_texts = ['South China','North East','North China','East China'] 
    recognized_ocr = []
    recognized_qr_texts = []
    PICK_TIMES = 999 

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
    mc = MechArm270('/dev/ttyACM0',115200) 
    mc.set_fresh_mode(0)

    mc.send_angles(angle_table["move_init"], 50)
    wait()

    # Register the Ctrl+C signal handler
    global running_flag
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    ##########################################################
    #  Function 1: Record information of three navigation points
    ##########################################################
    # ocr_recognized() # The visual recognition error is relatively large, so this feature is temporarily not in use.

    # Properly initialize the boxes_with_text list to ensure the data structure matches
    for i, text in enumerate(recognized_ocr_texts):
        if i < len(box_goals_1):
            box_info = {
                "text": text,
                "box_goals_1": box_goals_1[i],
                "box_pose_1":box_pose_1[i]
            }
            boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print("boxes with text",box)

    ##########################################################
    #  Function 2: Pick PICK_TIMES boxes and sort them
    ##########################################################
    timer=timer.TaskTimer()
    x_goal, y_goal, orientation_z, orientation_w = pack_goal # Navigate to the sorting shelf
    flag_feed_goalReached = False
    while not flag_feed_goalReached:
        print("Trying to reach pack_goal...")
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if not flag_feed_goalReached:
            print("Navigation failed, retrying...")
        time.sleep(2)  # Add a delay to prevent frequent calls
    for i in range(PICK_TIMES):
        timer.reset()
        # Initialize target_box variable
        target_box = None
        # if (initialized):
        #     initialized = False
        #     x_goal, y_goal, orientation_z, orientation_w = goal_1
        #     map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            

        # Check for the QR codes on top of of the sorting shelf
        target_info = proxy.aruco_rpc("check_box_qrcodes")
        target_box = target_info["target_id"]
        is_upper = target_info["is_upper"]

        # When target_box is None, it means there are no courier boxes above id3 and id4.
        if target_box is None:
            print("Start continuously checking for new courier boxes...")
            # Continuously check for new courier boxes until one is detected
            while target_box is None:
                print("Waiting for new courier box...")
                # Call check_box_qrcodes() function again to check
                target_info =  proxy.aruco_rpc("check_box_qrcodes")
                target_box = target_info["target_id"]
                is_upper = target_info["is_upper"]
                # If still no detected, wait for a while and check again later
                if target_box is None:
                    print("The courier box is still not detected, will check again in 3 seconds...")
                    time.sleep(3)

        print(f"Selected target: {target_box}, is_upper: {is_upper}")
        timer.lap("Check target type")
        print("calling rpc to align")
        safe_aruco_command(proxy,"align")
        map_navigation.pub_vel(0.0001,0, 0)
        time.sleep(0.5)
        map_navigation.pub_vel(0,0, 0)
        time.sleep(0.3)
        timer.lap("Approach target box")
        xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
        map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) 

        angle_pick = angle_table["pick_watch"]
        box_height = box_1_height
        recognized_qr_texts.append(pick(angle_pick, box_height, target_info)) 
        timer.lap("Pick target box")
        map_navigation.pub_vel(-0.1,0,0)
        time.sleep(4.5)
        map_navigation.pub_vel(0,0,0)

        map_navigation.pub_vel(0,0,-0.1)
        time.sleep(4)
        map_navigation.pub_vel(0,0,0)

        map_navigation.pub_vel(0.1,0,0)
        time.sleep(2)

    ##########################################################
    #  Function 3: Navigate to each city in recognized_qr_texts
    ##########################################################
        if recognized_qr_texts: 
            print("recognized_ocr_texts:",recognized_ocr_texts) #debug
            print("recognized_qr_texts:",recognized_qr_texts)   #debug 
            last_city = recognized_qr_texts[-1]
            region = city_to_region_mapping.get(last_city, "Unknown area") 
            print(f"City: {last_city}, Region: {region}")
            if region != "Unknown area":
                region_found = False
                for box in boxes_with_text:
                    print(f"Check box: {box['text']}")
                    if box["text"] == region:
                        region_found = True
                        box_goals_1 = box["box_goals_1"]

                        for target_num, goal in enumerate([box_goals_1], 1):

                            x_goal, y_goal, orientation_z, orientation_w = goal

                            print(f"Navigate to {region} target {target_num}: x={x_goal}, y={y_goal}, orientation_z={orientation_z}, orientation_w={orientation_w}")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        recognized_ocr.append(ocr_capture.start_capture())

                        print(f"recognized{recognized_ocr[-1]},{region}the package is about to be moved to{recognized_ocr[-1]}")

                        while True:
                            print("Starting alignment attempt...")
                            safe_aruco_command(proxy, "unload")
                        
                            # 1. Wait for the alignment process to finish
                            while True:
                                current_state = proxy.get_robot_state() 
                                if current_state != "ALIGNING":
                                    break
                                time.sleep(0.5)
                        
                            # 2. Check the result
                            if current_state == "IDLE":
                                print(">>> SUCCESS: Marker found and aligned! <<<")
                                break  # <--- This is the ONLY way to exit the loop and continue
                            
                            # 3. If we are here, it failed. Start Recovery.
                            print(f"Alignment Failed (State: {current_state}). Retrying recovery sequence...")
                            
                            # Rotate 180 degrees to 'shake' the localization or look around
                            map_navigation.pub_vel(0, 0, -0.1)
                            time.sleep(4)
                            map_navigation.pub_vel(0, 0, 0)
                        
                            # Re-navigate to the exact goal coordinates
                            print("Returning to goal coordinates...")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
                            
                            print("Navigation finished. Restarting alignment process...")
                            # Loop repeats from the top
                        map_navigation.set_pose(*box["box_pose_1"])
                        timer.lap("Approach target box")
                        load() 
                        timer.lap("Unload package")
                        timer.save_to_txt()
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(3.7)
                        map_navigation.pub_vel(0,0,0)
        x_goal, y_goal, orientation_z, orientation_w = pack_goal 
        flag_feed_goalReached = False
        while not flag_feed_goalReached:
            print("Trying to reach pack_goal...")
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        # time.sleep(1)
        # map_navigation.pub_vel(0,0.15,0)
        # time.sleep(1)
        # map_navigation.pub_vel(0,0,0)
        timer.lap("Return to initial position")

    sys.exit()  