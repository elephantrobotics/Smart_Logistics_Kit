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
import glob
import cv2
import cv2.aruco as aruco
import numpy as np

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

def check_box_qrcodes():
    """Check if there are express delivery boxes, by detecting id3 and id4 QR codes, and identify stacking relationships and left/right positions"""
    print("Start detecting express box QR codes...")
    
    # Initialize the camera (using the same camera as agv_aruco)
    cam = None
    try:
        # Use GStreamer pipeline to initialize the camera
        def gstreamer_pipeline(
            sensor_id=0,
            capture_width=3264,
            capture_height=2464,
            display_width=960,
            display_height=540,
            framerate=21,
            flip_method=0,
        ):
            return (
                "nvarguscamerasrc sensor-id=%d !"
                "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    sensor_id,
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
                )
            )
        
        cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        if not cam.isOpened():
            print("Unable to open camera")
            return {"target_id": None, "is_upper": False}
        
        # Set ARUCO dictionary and parameters
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        
        # Increase detection count to improve accuracy
        detect_count = 0
        max_detect_count = 10
        
        # Record detection count and position information
        id3_detected = 0  # Number of times id3 detected
        id4_detected = 0  # Number of times id4 detected
        
        # Record (X,Y) coordinates of id3 and id4
        id3_positions = []  # Record id3 (X,Y) coordinates
        id4_positions = []  # Record id4 (X,Y) coordinates
        
        # Record all markers and their positions in each frame
        all_markers_data = []
        
        while detect_count < max_detect_count:
            ret, frame = cam.read()
            if not ret or frame is None:
                detect_count += 1
                time.sleep(0.3)
                continue
            
            # Vertical mirror flip and grayscale
            frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Increase contrast to improve detection success rate
            gray = cv2.equalizeHist(gray)
            
            # Detect ARUCO markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Save the detection results of the current frame
            frame_markers = []
            
            # Check if id3 or id4 is detected
            current_detected_ids = []
            if ids is not None:
                for i, detected_id in enumerate(ids):
                    current_detected_ids.append(detected_id[0])
                    # Calculate the center point of the marker
                    if corners[i] is not None and len(corners[i]) > 0:
                        corner = corners[i][0]
                        center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
                        center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
                        
                        # Save to current frame data
                        frame_markers.append({
                            "id": detected_id[0],
                            "x": center_x,
                            "y": center_y
                        })
                        
                        if detected_id[0] == 3:
                            id3_detected += 1
                            id3_positions.append((center_x, center_y))
                            print(f"Detected id3 (ID detected for the{detect_count+1}time)，X Coordinate: {center_x}，Y Coordinate: {center_y}")
                        elif detected_id[0] == 4:
                            id4_detected += 1
                            id4_positions.append((center_x, center_y))
                            print(f"Detected id4 (ID detected for the{detect_count+1}time)，X Coordinate: {center_x}，Y Coordinate: {center_y}")    
            
            # Save frame data
            if frame_markers:
                all_markers_data.append(frame_markers)
            
            # Print all IDs detected in current frame for debugging
            print(f"ID detected for the{detect_count+1}time: {current_detected_ids}")
            
            detect_count += 1
            time.sleep(0.3)
        
        # Print statistics
        print(f"Detection completed. id3 detected {id3_detected} times, id4 detected {id4_detected} times")
        
        # Set detection threshold, need to detect at least 3 times to consider as existing
        min_detection_threshold = 3
        
        # Determine if express box exists based on detection count
        has_id3 = id3_detected >= min_detection_threshold
        has_id4 = id4_detected >= min_detection_threshold
        
        # Return result dictionary containing target ID and whether it's upper layer information
        result = {"target_id": None, "is_upper": False}
        
        # Count actual number of express boxes (deduplication)
        unique_id3_boxes = set()
        unique_id4_boxes = set()
        
        # Determine if it's the same box based on X/Y coordinate similarity (allow 10 pixel error)
        for x, y in id3_positions:
            found = False
            for box_x, box_y in list(unique_id3_boxes):
                if abs(x - box_x) < 10 and abs(y - box_y) < 10:
                    found = True
                    break
            if not found:
                unique_id3_boxes.add((x, y))
        
        for x, y in id4_positions:
            found = False
            for box_x, box_y in list(unique_id4_boxes):
                if abs(x - box_x) < 10 and abs(y - box_y) < 10:
                    found = True
                    break
            if not found:
                unique_id4_boxes.add((x, y))
        
        # Convert sets to lists for easier processing
        id3_boxes_list = list(unique_id3_boxes)
        id4_boxes_list = list(unique_id4_boxes)
        
        total_boxes = len(unique_id3_boxes) + len(unique_id4_boxes)
        print(f"Actually detected {len(unique_id3_boxes)} id3 boxes and {len(unique_id4_boxes)} id4 boxes, total {total_boxes} boxes")
        
        # If no targets detected
        if total_boxes == 0:
            print("No express boxes detected")
            return result
        
        # 1. Only one express box
        if len(unique_id3_boxes) == 1 and len(unique_id4_boxes) == 0:
            id3_y = id3_boxes_list[0][1]
            # Y coordinate below 250 is upper layer, above 250 is lower layer
            is_upper_id3 = id3_y < 250
            if is_upper_id3:
                print(f"Only one id3 box, y coordinate={id3_y}, in upper layer area, directly grab")
            else:
                print(f"Only one id3 box, y coordinate={id3_y}, in lower layer area, directly grab")
            result["target_id"] = "id3"
            result["is_upper"] = is_upper_id3
            return result
        elif len(unique_id4_boxes) == 1 and len(unique_id3_boxes) == 0:
            id4_y = id4_boxes_list[0][1]
            # Y coordinate below 250 is upper layer, above 250 is lower layer
            is_upper_id4 = id4_y < 250
            if is_upper_id4:
                print(f"Only one id4 box, y coordinate={id4_y}, in upper layer area, directly grab")
            else:
                print(f"Only one id4 box, y coordinate={id4_y}, in lower layer area, directly grab")
            result["target_id"] = "id4"
            result["is_upper"] = is_upper_id4
            return result
        
        # 2. Two express boxes
        elif total_boxes == 2:
            print("Detected 2 express boxes")
            
            # Case 2.1: One id3 and one id4
            if len(unique_id3_boxes) == 1 and len(unique_id4_boxes) == 1:
                print("One id3 and one id4")
                
                # Determine positions of id3 and id4
                id3_x, id3_y = id3_boxes_list[0]
                id4_x, id4_y = id4_boxes_list[0]
                
                # Check for stacking (Y coordinate difference greater than threshold)
                if abs(id3_y - id4_y) > 20:  # Stacking judgment threshold
                    # Has stacking, prioritize grabbing upper layer (smaller Y coordinate)
                    if id3_y < id4_y:
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"Detected stacking, id3 in upper layer ({id3_y} < {id4_y}), prioritize grabbing upper id3")
                    else:
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        print(f"Detected stacking, id4 in upper layer ({id4_y} < {id3_y}), prioritize grabbing upper id4")
                else:
                    # No stacking, use y coordinate 250 threshold to determine upper/lower layer
                    id3_x, id3_y = id3_boxes_list[0]
                    id4_x, id4_y = id4_boxes_list[0]
                    
                    # Y coordinate below 250 is upper layer, above 250 is lower layer
                    if id3_y < 250 and id4_y >= 250:
                        # id3 in upper, id4 in lower
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"Different IDs in same layer, id3 in upper layer ({id3_y} < 250), prioritize grabbing id3")
                    elif id4_y < 250 and id3_y >= 250:
                        # id4 in upper, id3 in lower
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        print(f"Different IDs in same layer, id4 in upper layer ({id4_y} < 250), prioritize grabbing id4")
                    else:
                        # Both in upper or both in lower, prioritize id3 by ID
                        result["target_id"] = "id3"
                        # Set is_upper flag based on actual area
                        if id3_y < 250 and id4_y < 250:
                            result["is_upper"] = True
                            print(f"Different IDs in same layer and both in upper layer, prioritize grabbing id3")
                        else:
                            result["is_upper"] = False
                            print(f"Different IDs in same layer and both in lower layer, prioritize grabbing id3")
                
                print(f"id3 coordinates: ({id3_x}, {id3_y}), id4 coordinates: ({id4_x}, {id4_y})")
                return result
            
            # Case 2.2: Two express boxes with same ID
            elif len(unique_id3_boxes) == 2:
                print("Two id3 boxes")
                
                # Check for stacking (Y coordinate difference greater than threshold)
                y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # Has stacking, prioritize grabbing upper layer (smaller Y coordinate)
                    upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                    print(f"Two id3 boxes stacked, prioritize grabbing upper id3")
                    result["target_id"] = "id3"
                    result["is_upper"] = True
                    return result
                else:
                    # No stacking, use y coordinate 250 threshold to determine upper/lower layer
                    # Y coordinate below 250 is upper layer, above 250 is lower layer
                    y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        # One in upper, one in lower, prioritize grabbing upper
                        upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                        print(f"Two id3 boxes not stacked, prioritize grabbing upper id3 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        # One in upper, one in lower, prioritize grabbing upper
                        upper_box = id3_boxes_list[1] if y2 < y1 else id3_boxes_list[0]
                        print(f"Two id3 boxes not stacked, prioritize grabbing upper id3 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    else:
                        # Same layer, prioritize grabbing left (smaller X coordinate)
                        left_box = min(id3_boxes_list, key=lambda pos: pos[0])
                        print(f"Two id3 boxes in same layer{', upper layer area' if y1 < 250 else ', lower layer area'}, prioritize grabbing left id3")
                        result["target_id"] = "id3"
                        # Set is_upper based on actual Y coordinate
                        result["is_upper"] = y1 < 250
                        return result
            
            elif len(unique_id4_boxes) == 2:
                print("Two id4 boxes")
                
                # Check for stacking (Y coordinate difference greater than threshold)
                y1, y2 = id4_boxes_list[0][1], id4_boxes_list[1][1]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # Has stacking, prioritize grabbing upper layer (smaller Y coordinate)
                    upper_box = id4_boxes_list[0] if y1 < y2 else id4_boxes_list[1]
                    print(f"Two id4 boxes stacked, prioritize grabbing upper id4")
                    result["target_id"] = "id4"
                    result["is_upper"] = True
                    return result
                else:
                    # No stacking, use y coordinate 250 threshold to determine upper/lower layer
                    y1, y2 = id4_boxes_list[0][1], id4_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        upper_box = id4_boxes_list[0] if y1 < y2 else id4_boxes_list[1]
                        print(f"Two id4 boxes not stacked, prioritize grabbing upper id4 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        upper_box = id4_boxes_list[1] if y2 < y1 else id4_boxes_list[0]
                        print(f"Two id4 boxes not stacked, prioritize grabbing upper id4 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        return result
                    else:
                        # Same layer, prioritize grabbing left (smaller X coordinate)
                        left_box = min(id4_boxes_list, key=lambda pos: pos[0])
                        print(f"Two id4 boxes in same layer{', upper layer area' if y1 < 250 else ', lower layer area'}, prioritize grabbing left id4")
                        result["target_id"] = "id4"
                        result["is_upper"] = y1 < 250
                        return result
        
        # 3. Three express boxes case
        elif total_boxes == 3:
            print("Detected 3 express boxes")
            
            # Collect all boxes position information
            all_boxes = []
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))
            for x, y in unique_id4_boxes:
                all_boxes.append((4, x, y))
            
            # Sort by Y coordinate to find upper layer boxes (smaller Y is more likely upper layer)
            all_boxes.sort(key=lambda box: box[2])
            
            # One side must be stacked, prioritize grabbing upper box
            upper_box = all_boxes[0]  # Box with smallest Y coordinate
            is_upper = upper_box[2] < 250  # Determine if actually in upper layer based on actual Y coordinate
            print(f"Prioritize grabbing upper box: id{upper_box[0]}, Y coordinate={upper_box[2]}, is upper layer area={is_upper}")
            result["target_id"] = f"id{upper_box[0]}"
            result["is_upper"] = is_upper
            return result
        
        # 4. Four express boxes case (two id3 and two id4)
        elif total_boxes == 4:
            print("Detected 4 express boxes (two id3 and two id4)")
            
            # Collect all boxes position information
            all_boxes = []
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))
            for x, y in unique_id4_boxes:
                all_boxes.append((4, x, y))
            
            # Sort by Y coordinate to find upper layer boxes (two with smaller Y coordinate)
            all_boxes.sort(key=lambda box: box[2])
            upper_boxes = all_boxes[:2]  # Two upper layer boxes
            
            # Check if upper layer are same ID
            upper_ids = [box[0] for box in upper_boxes]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"Both sides upper layer are id{upper_ids[0]}")
                # Prioritize grabbing left (smaller X coordinate)
                upper_boxes.sort(key=lambda box: box[1])
                target_id = f"id{upper_boxes[0][0]}"
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = target_id
                result["is_upper"] = is_upper
                print(f"Prioritize grabbing left upper {target_id}, is upper layer area={is_upper}")
                return result
            else:
                print("Both sides upper layer are different IDs")
                # Prioritize grabbing id3
                for box in upper_boxes:
                    if box[0] == 3:
                        print("Prioritize grabbing upper id3")
                result["target_id"] = "id3"
                result["is_upper"] = box[2] < 250
                print(f"Prioritize grabbing upper id3, is upper layer area={result['is_upper']}")
                return result
                # If no id3 in upper layer, grab first in upper layer
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = f"id{upper_boxes[0][0]}"
                result["is_upper"] = is_upper
                print(f"No id3 in upper layer, grab upper id{upper_boxes[0][0]}, is upper layer area={is_upper}")
                return result
        
        # Default case: return result based on priority
        if has_id3 and has_id4:
            print("Default priority: prioritize grabbing id3")
            result["target_id"] = "id3"
            # Calculate average Y coordinate to determine upper/lower layer
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            id4_avg_y = sum(y for x, y in id4_positions) / len(id4_positions) if id4_positions else 0
            result["is_upper"] = id3_avg_y > id4_avg_y
        elif has_id3:
            print("Default: only id3 detected, prioritize grabbing id3")
            result["target_id"] = "id3"
            # Determine if in upper layer based on actual Y coordinate
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            result["is_upper"] = id3_avg_y < 250
            print(f"id3 average Y coordinate={id3_avg_y}, is upper layer={result['is_upper']}")
        elif has_id4:
            print("Default: only id4 detected, prioritize grabbing id4")
            result["target_id"] = "id4"
            # Determine if in upper layer based on actual Y coordinate
            id4_avg_y = sum(y for x, y in id4_positions) / len(id4_positions) if id4_positions else 0
            result["is_upper"] = id4_avg_y < 250
            print(f"id4 average Y coordinate={id4_avg_y}, is upper layer={result['is_upper']}")
        
        print(f"Test Result: Target ID={result['target_id']}, Is Upper Layer={result['is_upper']}")
        return result
            
    except Exception as e:
        print(f"Error detecting QR code: {str(e)}")
        # By default, select id3 in case of an error and assume it is the upper layer
        return {"target_id": "id3", "is_upper": True}
    finally:
        # Ensure the camera is released
        if cam is not None:
            cam.release()
            try:
                cv2.destroyAllWindows()
            except:
                pass


def pick(angle_watch, box_height, pick_info=None, pick_times=1):
    """Pick up the express box function, supporting handling of stacked cases
    
    Parameters:
    angle_watch: The angle of the camera photo position
    box_height: The height of the pick-up
    pick_info: A dictionary containing the target ID and whether it is an upper layer
    pick_times: The number of pick-up times
    """
    global scanner 
    for i in range(pick_times): #i=1, express box is only picked up once
        # Reset scanner to ensure a clean state
        scanner = None
        scanner = QRCodeScanner(device_status['dctive_camera'])
        
        angles = angle_watch
        speed = 80
        mc.send_angles(angles, speed) # Camera shooting position
        wait(angles, 0) # Use exact angle position check

        retry_count = 0
        max_retries = 3
        success = False
        
        while retry_count < max_retries:
            try:
                qr_texts, tvecs = scanner.start_capture() # Get QR code city information and tvec displacement matrix
                time.sleep(1)
                print("qr_texts", qr_texts)
                print("tvecs", tvecs)

                if qr_texts is not None and tvecs is not None: 
                    curr_coords = mc.get_coords() # Get current pose
                    print("curr_coords", curr_coords)
                    time.sleep(2)
                    
                    while curr_coords is None:
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                        if curr_coords is not None:
                            break

                    # Matrix transformation to calculate pick-up coordinates
                    mat = homo_transform_matrix(*curr_coords) @ homo_transform_matrix(-10, -35, 10, 0, 0, 0)  # Hand-eye matrix
                    p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1]) # Convert to homogeneous coordinates
                    p_base = np.squeeze((mat @ p_end)[:-1]).astype(int) # Calculate base coordinates

                    # X error compensation
                    p_base[0] -= 50
                    # Y error compensation
                    p_base[1] += 40
                    # Z-axis fixed height
                    p_base[2] = box_height

                    new_coords = np.concatenate([p_base, curr_coords[3:]]) # Combine to form complete coordinates
                    print("move_coords", list(new_coords))
                    coords = list(new_coords)
                    speed = 60
                    mc.send_coords(coords, speed, 1)
                    wait(coords, 1) # Use exact coordinate position check

                    map_navigation.pump_on()
                    time.sleep(2)
                    print("pump_on")

                    curr_coords = mc.get_coords() # Get current pose
                    print("curr_coords", curr_coords)
                    time.sleep(2)
                    
                    while curr_coords is None:
                        time.sleep(0.5)
                        curr_coords = mc.get_coords()
                        print("coords_s is None")
                        if curr_coords is not None:
                            break
                    
                    # Adjust the lift height based on whether it is the top box
                    lift_height = 60 if (pick_info and pick_info.get("is_upper", False)) else 40
                    print(f"Lift height: {lift_height} {'(upper layer box)' if (pick_info and pick_info.get('is_upper', False)) else '(lower layer box)'}")
                    curr_coords[2] += lift_height  # z-axis lift
                    coords = curr_coords
                    speed = 40
                    mc.send_coords(coords, speed, mode=1) #z-axis lift
                    wait(coords, 1) # Use exact coordinate position check

                    angles = angle_table["pick_point2"]
                    speed = 50
                    mc.send_angles(angles, speed)
                    wait(angles, 0) # Use exact angle position check

                    angles = angle_table["place_init"]
                    speed = 80
                    mc.send_angles(angles, speed)
                    wait(angles, 0) # Use exact angle position check

                    coords_s = mc.get_coords() # Get current pose   
                    print(coords_s)
                    time.sleep(2)
                    
                    while coords_s is None:
                        time.sleep(0.5)
                        coords_s = mc.get_coords()
                        print("coords_s is None")
                        if coords_s is not None:
                            break

                    hight = 45
                    coords_s[2] -= hight
                    coords = coords_s
                    speed = 40
                    mc.send_coords(coords, speed, mode=1) #z-axis lowering
                    wait(coords, 1) # Use exact coordinate position check
                    map_navigation.pump_off()
                    time.sleep(2)
                    print("pump_off")

                    coords_s[2] += hight        
                    coords = coords_s
                    speed = 40
                    mc.send_coords(coords, speed, mode=1) #z-axis lift
                    wait(coords, 1) # Use exact coordinate position check

                    angles = angle_table["place_point4"]
                    speed = 50
                    mc.send_angles(angles, speed) # Transition point, prevent crashing into the box
                    wait(angles, 0) # Use exact angle position check

                    success = True
                    break

                else:
                    print("qr scanner failed, retrying...")
                    retry_count += 1
                    time.sleep(1)
                    if retry_count >= max_retries:
                        print(f"Reached maximum retry count ({max_retries}), failed to pick up")
                        # Try to initialize a new scanner
                        scanner = None
                        scanner = QRCodeScanner(device_status['dctive_camera'])
            except Exception as e:
                print(f"Error occurred during pick up process: {str(e)}, retrying...")
                retry_count += 1
                time.sleep(1)
                scanner = None
                scanner = QRCodeScanner(device_status['dctive_camera'])

    # Reset after completion
    angles = angle_table["move_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check
    return qr_texts

def load():
    angles = [0,0,0,0,0,0]
    speed = 60
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check

    angles = angle_table["place_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check

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
    coords = coords_s
    speed = 40
    mc.send_coords(coords, speed, mode=1) #z-axis lowering
    wait(coords, 1) # Use exact coordinate position check
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
    mc.send_coords(coords, speed, mode=1) #z-axis lifting
    wait(coords, 1) # Use exact coordinate position check

    angles = angle_table["place_point4"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check

    angles = angle_table["place_point2"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check

    angles = angle_table["place_point3"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check
    
    map_navigation.pump_off()
    time.sleep(2)

    # End reset
    angles = angle_table["move_init"]
    speed = 50
    mc.send_angles(angles, speed)
    wait(angles, 0) # Use exact angle position check

def ocr_recognized():
        
    # List of target points, organized according to the order you provided
    goals_sequence = [
        (box_goals_0[0], box_goals_2[0], box_goals_2[1]),  # box_goals_0 Point 1 -> box_goals_2 Point 1, 2
        (box_goals_0[1], box_goals_2[2], box_goals_2[3]),  # box_goals_0 Point 2 -> box_goals_2 Point 3, 4
        (box_goals_0[2], box_goals_2[4])                   # box_goals_0 Point 3 -> box_goals_2 Point 5
    ]

    # Traverse the target point order for navigation
    for goal_set in goals_sequence:
        for i, goal in enumerate(goal_set):
            
            # Target coordinates
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

def wait(data=None, ids=0, max_same_data_count=50):
    """
    Enhanced wait function, compatible with the original simple calls, while also supporting precise position checks
    :param data: Angle or coordinate data, default is None (only checks if movement has stopped)
    :param ids: Angle-0, Coordinate-1, default is 0
    :param max_same_data_count: Maximum count threshold for consecutive identical data
    """
    import traceback
    import time
    
    # If no data parameter is provided, the original simple waiting logic will be used.
    if data is None:
        time.sleep(0.3)
        state = mc.is_moving()
        while(state != 0):
            state = mc.is_moving()
            time.sleep(0.1)
        return
    
    # Otherwise, use the precise checking logic of check_position
    try:
        same_data_count = 0
        last_data = None
        start_time = time.time()
        while True:
            # Timeout Detection
            if (time.time() - start_time) >= 5:
                break
            res = mc.is_in_position(data, ids)
            
            # Consecutive identical data detection
            if data == last_data:
                same_data_count += 1
            else:
                same_data_count = 0

            last_data = data
            
            # Exit conditions: reaching the target position (res==1) or consecutive identical data threshold
            if res == 1 or same_data_count >= max_same_data_count:
                break
            time.sleep(0.1)
    except Exception as e:
        e = traceback.format_exc()
        print(e)

if __name__ == '__main__':

    # Define the target positions for id3 and id4
    box_goals_0 = [
        [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919],#中间一号点,姿态朝前
        [-1.1358978748321533,1.0654418468475342,0.8887916047179362,0.45831155711253446],#中间二号点,姿态朝前      
        [-1.7721543312072754,1.5437819957733154,0.8958855713120555,0.4442848670784004] #中间三号点,姿态朝前
    ]

    box_goals_1 = [
        [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766],#1号盒子位姿
        [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218],#2号盒子位姿
        [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766],
        [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218]
    ]
    
    # Define the target positions for id3 and id4
    box_goal_id3 = [0.08485770225524902,-0.11438778042793274,-0.7170147446397785,0.6970580004340766]  # id3的目标点位置
    box_goal_id4 = [0.7079846858978271,-0.13074418902397156,-0.6723159317868799,0.7402643364809218]  # id4的目标点位置

    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#中间一号点,姿态朝前
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#中间一号点,姿态朝后
    
    pack_goal = [0.4767872333526611,0.04532311737537384,0.06401975195944533,0.9979486316234173]#快递分拣盒附近
    charge_goal =[-0.5688837170600891,-0.31650811433792114,0.4588518939959322,0.8885127682686084]

    pack_pose = [0.8529961109161377,0.050533026456832886,0.0112260354743728,0.9999369860783869,0.06853892326654787]

    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 10.2, -21.88, 0.96, 90.79, 0.0],    #Camera photo position 2, center point of the delivery box
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #Pick transition point
    "place_init":[-93.6, 1.93, 6.24, -0.17, 75.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #Place box position
    "place_point4":[-93.36, 20.03, -22.85, -3.07, 89, 1.46]
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

    # initialized = True # Initial navigation action
    USB_CAN_Enable = False # Whether to enable communication with the charging device
    box_2_height = 60  #Height of the second layer of the delivery box 101, demo2 140
    box_1_height = 17   #Height of the first layer of the delivery box 60, demo2 90

    boxes_with_text = []
    recognized_ocr_texts = ['South China','North East','North China','East China'] #Fixed delivery sorting points
    recognized_ocr = [] # OCR-recognized courier sorting point
    recognized_qr_texts = []
    PICK_TIMES = 999  # Loop pick times

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

    plist = get_port_list()
    print(plist)
    # mc = MechArm270('/dev/ttyACM0',115200) # Connect the robotic arm
    mc = MechArm270('/dev/ttyACM0',115200,debug=1) # Connect the robotic arm and enable debug mode
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
        print(box)

    ##########################################################
    # # Function 2: Loop pick PICK_TIMES times, each time only pick one box, then sort the box
    ##########################################################
    for i in range(PICK_TIMES):
        # Initialize the target_box variable
        target_box = None
        # if (initialized):
        #     initialized = False
        #     x_goal, y_goal, orientation_z, orientation_w = goal_1
        #     map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
        x_goal, y_goal, orientation_z, orientation_w = pack_goal # Navigate to the delivery sorting shelf
        flag_feed_goalReached = False
        while not flag_feed_goalReached:
                print("Trying to reach pack_goal...")
                flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
                if not flag_feed_goalReached:
                    print("Navigation failed, retrying...")
                    time.sleep(2)  # Add a delay to prevent frequent calls

            # After reaching pack_goal, detect QR codes on id3 and id4
        target_info = check_box_qrcodes()
        target_box = target_info["target_id"]
        is_upper = target_info["is_upper"]
        print(f"Select the target based on the QR code detection results: {target_box}, whether the higher level: {is_upper}")
        # When target_box is None, it means that there is no delivery box on id3 and id4
        if target_box is None:
            print("The car is at the pack_goal position, start continuously detecting whether there is a new delivery box put in...")
            # Continuously detect until a new delivery box is detected
            while target_box is None:
                print("Wait for a while to detect a new delivery box...")
                # Call the check_box_qrcodes() function again to detect
                target_info = check_box_qrcodes()
                target_box = target_info["target_id"]
                is_upper = target_info["is_upper"]
                # If still not detected, wait for a while and detect again
                if target_box is None:
                    print("Still not detected a delivery box, wait for 3 seconds and detect again...")
                    time.sleep(3)
            print(f"Detected a new delivery box, select the target: {target_box}, whether the higher level: {is_upper}")

        print("python agv_aruco_1")
        os.system('python agv_aruco_1.py') 

        xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
        map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) #amcl重定位

        # Capture based on the number of cycles, fixed camera shooting position, and aspiration height
        angle_pick = angle_table["pick_watch"]
        # Dynamically set the grabbing height based on whether it is an upper box
        if is_upper:
            box_height = box_2_height
            print(f"Set the grabbing height for the upper box: {box_height}")
        else:
            box_height = box_1_height
            print(f"Set the grabbing height for the lower box: {box_height}")

        recognized_qr_texts.append(pick(angle_pick, box_height, target_info))  # Send the camera joint angles, Z-axis height, and target information, capture and return the recognized text

        # x shifts 1 second
        map_navigation.pub_vel(-0.1,0,0)
        time.sleep(4.5)
        map_navigation.pub_vel(0,0,0)

        # Right turn 180°
        map_navigation.pub_vel(0,0,-0.1)
        time.sleep(4)
        map_navigation.pub_vel(0,0,0)

        # x shifts 1 second
        map_navigation.pub_vel(0.1,0,0)
        time.sleep(2)

    ##########################################################
    # # Function 3: Loop navigate to each city in the recognized_qr_texts list
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
                region_found = False
                for box in boxes_with_text:
                    print(f"Check the box: {box['text']}")
                    if box["text"] == region:
                        region_found = True
                        # Get the target location for this area
                        box_goals_1 = box["box_goals_1"]

                        # Loop navigate to each target point and direction
                        for target_num, goal in enumerate([box_goals_1], 1):
                            # Target coordinates
                            x_goal, y_goal, orientation_z, orientation_w = goal

                            print(f"Navigate to the target {target_num} in {region}: x={x_goal}, y={y_goal}, direction z={orientation_z}, direction w={orientation_w}")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        recognized_ocr.append(ocr_capture.start_capture())

                        print(f"Recognized the text: {recognized_ocr[-1]}, {region} express will be delivered to {recognized_ocr[-1]}")

                        print("python agv_aruco_2")
                        os.system('python agv_aruco_2.py')    # Navigation target point
                        
                        load()  # Navigate to all target points and deliver the express
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(3.7)
                        map_navigation.pub_vel(0,0,0)

    sys.exit()  # End the program
