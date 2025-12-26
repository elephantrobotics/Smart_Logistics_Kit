#coding=UTF-8
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

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

cam = None

def init_camera():
    global cam
    if cam is None:
        cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        return cam.isOpened() if cam else False
    return True

def close_camera():
    global cam
    if cam is not None:
        cam.release()
        cam = None
        try:
            cv2.destroyAllWindows()
        except:
            pass
        print("Camera closed successfully")
        return True
    return False

# The camera initialization will be controlled by the caller and will no longer be initialized automatically.

font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)

# Predefined constant parameters to avoid accessing the camera at the module level.
CAM_WIDTH = 960
CAM_HEIGHT = 540
CAM_FPS = 21

# Calculate the default center point and focal length
center = (CAM_WIDTH / 2, CAM_HEIGHT / 2)
focal_length = CAM_WIDTH

# Camera internals: intrinsic parameter matrix of the camera
camera_matrix = np.array([[785.855437,  0.000000,   451.670922], 
                          [0.000000,    584.820336, 259.056856],
                          [0.000000,    0.000000,   1.000000]])

# Distortion coefficients matrix
dist_coeffs = np.array(([[0.095135, -0.109279, -0.002513,  -0.002418, 0.000000]]))

print(camera_matrix,dist_coeffs)

# D = [0.09513462081295623, -0.10927855766094025, -0.002513183030897191, -0.002417838068546282, 0.0]
# K = [785.8554366929662, 0.0, 451.6709224755481, 0.0, 584.8203363630505, 259.056855562218, 0.0, 0.0, 1.0]
# R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P = [802.9904422167656, 0.0, 449.2579140738804, 0.0, 0.0, 598.1678611537059, 257.74078396240856, 0.0, 0.0, 0.0, 1.0, 0.0]
# # oST version 5.0 parameters


# [image]

# width
# 960

# height
# 540

# [narrow_stereo]

# camera matrix
# 785.855437 0.000000 451.670922
# 0.000000 584.820336 259.056856
# 0.000000 0.000000 1.000000

# distortion
# 0.095135 -0.109279 -0.002513 -0.002418 0.000000

# rectification
# 1.000000 0.000000 0.000000
# 0.000000 1.000000 0.000000
# 0.000000 0.000000 1.000000

# projection
# 802.990442 0.000000 449.257914 0.000000
# 0.000000 598.167861 257.740784 0.000000
# 0.000000 0.000000 1.000000 0.000000

# Window creation will be done in the displayFrame function to avoid module-level initialization.

marker_length = 0.04   # -- Here, the measurement unit is metre.0.055 is for orgianl big

# use to get the attitude in terms of euler 321
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

pose_data = [None, None, None, None, None, None]
_id = [0]
pose_data_dict = {}

x = 0
y = 0
theta = 0

def _is_rotation_matrix(R):
        """
        Checks if a matrix is a valid rotation matrix.

        :param R:    rotation matrix
        :return:     [bool] True or False
        """
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


def _rotation_matrix_to_euler_angles(R):
        """
        Calculates rotation matrix to euler angles

        :param R:     rotation matrix
        :return:      [np.array] roll, pitch, yaw
        """
        assert (_is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])


def _detect(corners, ids, imgWithAruco):
        """
        Show the Axis of aruco and return the x,y,z(unit is cm), roll, pitch, yaw

        :param corners:        get from cv2.aruco.detectMarkers()
        :param ids:            get from cv2.aruco.detectMarkers()
        :param imgWithAruco:   assign imRemapped_color to imgWithAruco directly
        :return:               x,y,z (units is cm), roll, pitch, yaw (units is degree)
        """
        try:
            # Check the validity of input parameters
            if corners is None or len(corners) == 0 or imgWithAruco is None:
                return None
            
            x1 = (int(corners[0][0][0][0]), int(corners[0][0][0][1]))
            x2 = (int(corners[0][0][1][0]), int(corners[0][0][1][1]))
            x3 = (int(corners[0][0][2][0]), int(corners[0][0][2][1]))
            x4 = (int(corners[0][0][3][0]), int(corners[0][0][3][1]))
            # Drawing detected frame white color
            # OpenCV stores color images in Blue, Green, Red
            cv2.line(imgWithAruco, x1, x2, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x2, x3, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x3, x4, (255, 0, 0), 1)
            cv2.line(imgWithAruco, x4, x1, (255, 0, 0), 1)
            # font type hershey_simpex
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(imgWithAruco, 'C1', x1, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C2', x2, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C3', x3, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(imgWithAruco, 'C4', x4, font, 1, (255, 255, 255), 1,
                        cv2.LINE_AA)
            if ids is not None:   # if aruco marker detected
                try:
                    # Use different size parameters based on the QR code ID
                    # ids is an array, and the first element is the ID of the tag
                    current_marker_length = 0.03 if ids[0] in [3, 4] else 0.04
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, current_marker_length, camera_matrix, dist_coeffs)
                    for i in range(rvec.shape[0]):
                        # Use the current marker length for pose estimation
                        imgWithAruco = cv2.drawFrameAxes(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, current_marker_length)

                    # --- The midpoint displays the ID number
                    cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                                 int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                    # Use the incoming imgWithAruco instead of the global frame
                    cv2.putText(imgWithAruco, "id=" + str(ids), cornerMid,
                                font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                    rvec = rvec[0][0]
                    tvec = tvec[0][0]
                    # -- Obtain the rotation matrix tag->camera
                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T
                    # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                    roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)

                    pose_data[0] = tvec[0] * 100
                    pose_data[1] = tvec[1] * 100
                    pose_data[2] = tvec[2] * 100
                    pose_data[3] = math.degrees(roll_marker)
                    pose_data[4] = math.degrees(pitch_marker)
                    pose_data[5] = math.degrees(yaw_marker)

                    # Use a tuple as a key, because arrays cannot be dictionary keys
                    pose_data_dict[str(ids)] = pose_data.copy()

                    roll_deg = math.degrees(roll_marker)
                    pitch_deg = math.degrees(pitch_marker)
                    yaw_deg = math.degrees(yaw_marker)

                    if abs(yaw_deg) % 90.0 < 30:
                        return [tvec[0] * 100, tvec[1] * 100, tvec[2] * 100, roll_deg, pitch_deg, yaw_deg, cornerMid]
                except Exception:
                    pass
        except Exception:
            pass
        
        # If no valid marker is detected, reset pose_data and return None
        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
        pose_data[3] = None
        pose_data_dict[0] = pose_data.copy()
        return None

def displayFrame(frame_input):
    try:
        # Make sure the window exists
        if not cv2.getWindowProperty("show", cv2.WND_PROP_VISIBLE):
            cv2.namedWindow("show", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("show", frame_input)
        # Use a smaller waitKey value to reduce latency while maintaining window responsiveness
        cv2.waitKey(1)
    except Exception:
        # Ignore display-related errors to ensure program continuation
        pass

def getArucoCode(display_mode = True ):
    global cam
    # Check if the camera is closed or not initialized
    if cam is None or not cam.isOpened():
        return None
    
    # read frame once
    frame_makers = None
    try:
        ret, frame = cam.read() #Get the camera's data stream
        if not ret or frame is None:
            return None
        
        frame = cv2.flip(frame,-1) #Vertically flip the frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Grayscale conversion
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #Set the predefined dictionary
        
        parameters = cv2.aruco.DetectorParameters() #Initialize the detector parameters with default values
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标

        if ids is not None:
            try:
                frame_makers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                ids_len = len(ids)

                res = []
                i = 0
                if ids_len > 0:
                    for l in range(ids_len):
                        try:
                            aruco_res = _detect(corners[i:i+1], ids[i], frame) #Input the four corner coordinates to calculate and return x,y,z (units is cm), roll, pitch, yaw (units is degree),Aruco id
                            if aruco_res != None:
                                res.append(aruco_res)
                        except Exception:
                            pass  # Ignore errors for individual marker detections
                        i += 1
                        if display_mode and frame_makers is not None:
                            displayFrame(frame_makers)
                return (res, ids)
            except Exception:
                # If an error occurs during processing, try to display the original frame
                if display_mode:
                    displayFrame(frame)
                return None

        else:
            # no data detected
            frame_makers = frame.copy()
            if display_mode:
                displayFrame(frame_makers)
            return None
    except Exception:
        return None
    
    
def process_qr_data():
    """
    Processing QR code data, determining which target data to return based on the existence of the delivery box and the stacking relationship.

    Logic rules:
    1. If there is only one delivery box, directly grab the detected delivery box.
    2. If there are two delivery boxes:
    - One is id3 and the other is id4: prioritize grabbing id3.
    - If both have the same ID: prioritize grabbing the left one; if stacked, prioritize the upper one.
    3. If there are three delivery boxes: one side must be stacked, prioritize grabbing the upper delivery box.
    4. If there are four delivery boxes: both sides are stacked, prioritize the upper one; for the same ID, grab the left side upper box, for different IDs, prioritize id3 on top.

    Return:
    If a grabbable delivery box is found: return (_z, _ry, _perc), where _z is depth, _ry is pitch angle, and _perc is the normalized center value.
    If no grabbable delivery box is found: return -1.
    """
    global cam
    # Check if the camera is closed or not initialized
    if cam is None or not cam.isOpened():
        return -1
    
    data_ = getArucoCode(True)
    # Check if the data is valid
    if data_ is not None and len(data_) > 1 and data_[1] is not None:
        results, ids = data_
        
        if results == []:
            return -1   #The QR code's marker_length needs to be large, and the marker_length parameter must be set correctly; otherwise, there will be no pose information.
        
        # Create a mapping from ID to index for quick lookup
        id_to_index = {}
        for i, marker_id in enumerate(ids):
            id_to_index[marker_id[0]] = i
        
        # Check if a specific ID is detected
        has_id3 = 3 in id_to_index and id_to_index[3] < len(results)
        has_id4 = 4 in id_to_index and id_to_index[4] < len(results)
        
        # Collect all ID3 and ID4 tag information
        id3_markers = []
        id4_markers = []
        
        for i, marker_id in enumerate(ids):
            mid = marker_id[0]
            if i < len(results):
                if mid == 3:
                    x, y = results[i][6][0], results[i][6][1]
                    id3_markers.append((i, x, y))  # (Index, X Coordinate, Y Coordinate)
                elif mid == 4:
                    x, y = results[i][6][0], results[i][6][1]
                    id4_markers.append((i, x, y))  # (Index, X Coordinate, Y Coordinate)
        
        total_id3 = len(id3_markers)
        total_id4 = len(id4_markers)
        
        print(f"Test Results: Number of id3={total_id3}, Number of id4={total_id4}")
        
        # 1. Only one delivery box
        if total_id3 == 1 and total_id4 == 0:
            y_id3 = id3_markers[0][2]
            # Y-coordinates below 250 are the upper layer, and above 250 are the lower layer.
            if y_id3 < 250:
                print(f"Only one id3 delivery box, y coordinate={y_id3}, in the upper layer, directly grab")
            else:
                print(f"Only one id3 delivery box, y coordinate={y_id3}, in the lower layer, directly grab")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif total_id4 == 1 and total_id3 == 0:
            y_id4 = id4_markers[0][2]
            # Y-coordinates below 250 are the upper layer, and above 250 are the lower layer.
            if y_id4 < 250:
                print(f"Only one id4 delivery box, y coordinate={y_id4}, in the upper layer, directly grab")
            else:
                print(f"Only one id4 delivery box, y coordinate={y_id4}, in the lower layer, directly grab")
            i = id4_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # 2. The situation with the two delivery boxes
        elif (total_id3 + total_id4) == 2:
            print("Detected 2 delivery boxes")
            
            # 2.1 One id3 and one id4
            if total_id3 == 1 and total_id4 == 1:
                print("One id3 and one id4")
                # Check for stacking (Y coordinate difference greater than threshold)
                y_id3 = id3_markers[0][2]
                y_id4 = id4_markers[0][2]
                
                if abs(y_id3 - y_id4) > 20:  # Stacking Judgment Threshold
                    # There is stacking; prioritize grabbing the upper layer (smaller Y coordinate).
                    if y_id3 < y_id4:
                        i = id3_markers[0][0]
                        print("Different IDs are stacked, with id3 on the top layer, prioritize capturing id3")
                    else:
                        i = id4_markers[0][0]
                        print("Different IDs stacked, id4 is on top, prioritize grabbing id4")
                else:
                    # No stacking; use y coordinate 250 threshold to judge upper and lower layers
                    y_id3 = id3_markers[0][2]
                    y_id4 = id4_markers[0][2]
                    
                    # Y-coordinates below 250 are the upper layer, and above 250 are the lower layer.
                    if y_id3 < 250 and y_id4 >= 250:
                        # id3 is in the upper layer, id4 is in the lower layer.
                        i = id3_markers[0][0]
                        print("Different IDs are in the same layer, id3 is in the upper layer, prioritize capturing id3")
                    elif y_id4 < 250 and y_id3 >= 250:
                        # id4 is in the upper layer, id3 is in the lower layer.
                        i = id4_markers[0][0]
                        print("Different IDs are in the same layer, id4 is in the upper layer, prioritize capturing id4")
                    else:
                        # 都在上层或都在下层，按照ID优先级，优先抓Whether they are all on the upper level or all on the lower level, according to ID priority, prioritize capturing id3.取id3
                        i = id3_markers[0][0]
                        # Clearly distinguish regions
                        if y_id3 < 250 and y_id4 < 250:
                            print("Different IDs on the same layer and both in the upper area, prioritize capturing id3")
                        else:
                            print("Different IDs on the same layer and both in the lower area, prioritize capturing id3")
                
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
            
            # 2.2 Two courier boxes with the same ID
            elif total_id3 == 2:
                print("Two id3 delivery boxes")
                # Check for stacking (Y-coordinate difference exceeds the threshold)
                y1, y2 = id3_markers[0][2], id3_markers[1][2]
                if abs(y1 - y2) > 20:  # Stacking Judgment Threshold
                    # There is stacking; prioritize grabbing the upper layer (smaller Y coordinate).
                    i = id3_markers[0][0] if y1 < y2 else id3_markers[1][0]
                    print("Two id3 delivery boxes are stacked, prioritize capturing the upper layer id3")
                else:
                    # Use a y-coordinate threshold of 250 to determine upper and lower levels.
                    # Y-coordinates below 250 are the upper layer, and above 250 are the lower layer.
                    if y1 < 250 and y2 >= 250:
                        # id3 is in the upper layer, id3 is in the lower layer.
                        i = id3_markers[0][0]
                        print("Two id3 delivery boxes are not in the same layer, prioritize capturing the upper layer id3")
                    elif y2 < 250 and y1 >= 250:
                        # id3 is in the upper layer, id3 is in the lower layer.
                        i = id3_markers[1][0]
                        print("Two id3 delivery boxes are not in the same layer, prioritize capturing the upper layer id3")
                    else:
                        # Both id3 are in the upper layer or both are in the lower layer.
                        # Prioritize grabbing the left one (smaller X coordinate).
                        x1, x2 = id3_markers[0][1], id3_markers[1][1]
                        i = id3_markers[0][0] if x1 < x2 else id3_markers[1][0]
                        # Clearly distinguish regions
                        if y1 < 250 and y2 < 250:
                            print("Two id3 delivery boxes are in the same layer and both in the upper area, prioritize capturing the left id3")
                        else:
                            print("Two id3 delivery boxes are in the same layer and both in the lower area, prioritize capturing the left id3")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
            
            elif total_id4 == 2:
                print("Two id4 delivery boxes")
                # Check for stacking (Y-coordinate difference exceeds the threshold)
                y1, y2 = id4_markers[0][2], id4_markers[1][2]
                if abs(y1 - y2) > 20:  # Stacking Judgment Threshold
                    # There is stacking; prioritize grabbing the upper layer (smaller Y coordinate).
                    i = id4_markers[0][0] if y1 < y2 else id4_markers[1][0]
                    print("Two id4 delivery boxes are stacked, prioritize capturing the upper layer id4")
                else:
                    # Use a y-coordinate threshold of 250 to determine upper and lower levels.
                    # Y-coordinates below 250 are the upper layer, and above 250 are the lower layer.
                    if y1 < 250 and y2 >= 250:
                        # id4 is in the upper layer, id4 is in the lower layer.
                        i = id4_markers[0][0]
                        print("Two id4 delivery boxes are not in the same layer, prioritize capturing the upper layer id4")
                    elif y2 < 250 and y1 >= 250:
                        # id4 is in the upper layer, id4 is in the lower layer.
                        i = id4_markers[1][0]
                        print("Two id4 delivery boxes are not in the same layer, prioritize capturing the upper layer id4")
                    else:
                        # Both id4 are in the upper layer or both are in the lower layer.
                        # Prioritize grabbing the left one (smaller X coordinate).
                        x1, x2 = id4_markers[0][1], id4_markers[1][1]
                        i = id4_markers[0][0] if x1 < x2 else id4_markers[1][0]
                        # Clearly distinguish regions
                        if y1 < 250 and y2 < 250:
                            print("Two id4 delivery boxes are in the same layer and both in the upper area, prioritize capturing the left id4")
                        else:
                            print("Two id4 delivery boxes are in the same layer and both in the lower area, prioritize capturing the left id4")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
        
        # 3. The Case of the Three Delivery Boxes
        elif (total_id3 + total_id4) == 3:
            print("Detected 3 delivery boxes, one side must be stacked")
            
            # Collect the location information of all the boxes
            all_markers = []
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))  # (Index, ID, X Coordinate, Y Coordinate)
            for i, x, y in id4_markers:
                all_markers.append((i, 4, x, y))
            
            # Sort by Y coordinate to find the upper box (the smaller the Y value, the more likely it is on top)
            all_markers.sort(key=lambda m: m[3])
            
            # Prioritize grabbing the upper layer delivery box
            upper_marker = all_markers[0]
            i = upper_marker[0]
            print(f"Prioritize grabbing the upper layer delivery box: id{upper_marker[1]}")
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # 4. The Case of the Four Delivery Boxes (Two id3 and Two id4)
        elif (total_id3 + total_id4) == 4 and total_id3 == 2 and total_id4 == 2:
            print("Detected 4 delivery boxes, one side must be stacked")
            
            # Collect the location information of all the boxes
            all_markers = []
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))  # (Index, ID, X Coordinate, Y Coordinate)
            for i, x, y in id4_markers:
                all_markers.append((i, 4, x, y))
            
            # Sort by Y coordinate to find the upper boxes (the smaller the Y value, the more likely it is on top)
            all_markers.sort(key=lambda m: m[3])
            upper_markers = all_markers[:2]  # The upper two boxes  
            
            # Check if the upper boxes are of the same ID
            upper_ids = [m[1] for m in upper_markers]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"Both upper boxes are id{upper_ids[0]}")
                # Prioritize grabbing the left one (smaller X coordinate)
                upper_markers.sort(key=lambda m: m[2])
                target_marker = upper_markers[0]
                i = target_marker[0]
                print(f"Prioritize grabbing the left upper box: id{target_marker[1]}")
            else:
                print("The upper boxes are of different IDs")   
                # Prioritize grabbing id3
                target_marker = None
                for m in upper_markers:
                    if m[1] == 3:
                        target_marker = m
                        break
                
                if target_marker:
                    i = target_marker[0]
                    print("Prioritize grabbing the upper layer id3")
                else:
                    # If there is no id3 in the upper layer, grab the first one
                    i = upper_markers[0][0]
                    print(f"Upper layer has no id3, prioritize grabbing the upper layer id{upper_markers[0][1]}")
            
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # Default case: return results based on priority
        if has_id3:
            print("Default priority: prioritize grabbing id3")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif has_id4:
            print("Default priority: prioritize grabbing id4")
            i = id4_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # If no delivery boxes are detected, return -1
        print("No delivery boxes detected")
        return -1
    
    # If no markers are detected, return -1
    print("No markers detected")
    return -1

def process_qr_data_2():
    global cam
    # Check if the camera is closed or not initialized
    if cam is None or not cam.isOpened():
        return -1
    
    data_ = getArucoCode(True)
    # print("data_",data_)
    # Example: data_ ([[13.190542591653859, 0.6577785493956305, 30.57489950772101, 56.677235927555834, -10.703176777425517, -17.524720968623765, (892, 292)]], array([[2]], dtype=int32))
    
    if data_ is not None:
        if data_[0] == []:
            return -1   #The QR code's marker_length needs to be large, and the marker_length parameter must be set correctly; otherwise, there will be no pose information.

        _z = data_[0][0][2]
        _ry = data_[0][0][4]
        _perc = data_[0][0][6][0]/960.0 # Normalized [0,1], (892, 292) as the coordinates of the QR code center
        return (_z, _ry, _perc) # Return depth z, pitch angle, and normalized center position percentage of the QR code
    else:
        return -1

if __name__=='__main__':
    pass
