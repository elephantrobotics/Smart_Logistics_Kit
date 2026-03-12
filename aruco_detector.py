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
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=True max-buffers=1 emit-signals=True"
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

# Camera initialization will be controlled by the caller and will no longer be automatic.
font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)

# Predefined constant parameters prevent access to the camera at the module level.
CAM_WIDTH = 960
CAM_HEIGHT = 540
CAM_FPS = 21

# Default values ​​for calculating center point and focal length
center = (CAM_WIDTH / 2, CAM_HEIGHT / 2)
focal_length = CAM_WIDTH

# Camera internals Camera internal parameter matrix
camera_matrix = np.array([[785.855437,  0.000000,   451.670922], 
                          [0.000000,    584.820336, 259.056856],
                          [0.000000,    0.000000,   1.000000]])

#Distortion coefficient matrix
dist_coeffs = np.array(([[0.095135, -0.109279, -0.002513,  -0.002418, 0.000000]]))

# print(camera_matrix,dist_coeffs)

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

# Window creation will occur in the displayFrame function, avoiding module-level initialization.

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
            # Check the validity of the input parameters.
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
                    # Use different size parameters based on the QR code ID.
                    # ids is an array, the first element of which is the ID of the tag.
                    current_marker_length = 0.03 if ids[0] in [3, 4] else 0.04
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, current_marker_length, camera_matrix, dist_coeffs)
                    for i in range(rvec.shape[0]):
                        # Use the size parameters corresponding to the current QR code
                        imgWithAruco = cv2.drawFrameAxes(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, current_marker_length)

                    # --- The midpoint displays the ID number
                    cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                                 int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                    # Use the passed-in `imgWithAruco` instead of the global `frame`.
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

                    # Use tuples as keys because arrays cannot be used as dictionary keys.
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
        
        # If an error occurs or a valid flag is not detected, reset pose_data and return None.
        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
        pose_data[3] = None
        pose_data_dict[0] = pose_data.copy()
        return None

def displayFrame(frame_input):
    try:
        # Ensure the window exists
        if not cv2.getWindowProperty("show", cv2.WND_PROP_VISIBLE):
            cv2.namedWindow("show", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("show", frame_input)
        # Use a smaller waitKey value to reduce latency, but maintain window responsiveness.
        cv2.waitKey(1)
    except Exception:
        # Ignore any related errors and ensure the program continues running.
        pass

def getArucoCode(display_mode = True ):
    global cam
    # Check if the camera is turned off or not initialized.
    if not init_camera():
        print("Camera failed!")
        return None
    
    # read frame once
    frame_makers = None
    try:
        ret, frame = cam.read() #Acquire camera data stream
        if not ret or frame is None:
            print("get aruco code none",ret,cam)
            return None
        
        frame = cv2.flip(frame,-1) #Vertical mirror flip
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Grayscale
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #Set a predefined dictionary
        
        parameters = cv2.aruco.DetectorParameters() #Initialize detector parameters using default values.
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #The aruco.detectMarkers() function can be used to detect markers, returning their IDs and the coordinates of the four corner points of the marker.

        if ids is not None:
            try:
                frame_makers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                ids_len = len(ids)

                res = []
                if ids_len > 0:
                    for i in range(ids_len):
                        try:
                            aruco_res = _detect(corners[i:i+1], ids[i], frame) #Input the coordinates of four corner points, calculate and return x, y, z (units are cm), roll, pitch, yaw (units are degrees), and arcade id.
                            if aruco_res != None:
                                res.append(aruco_res)
                        except Exception:
                            pass  # Errors in ignoring single marker detection
                        if display_mode and frame_makers is not None:
                            displayFrame(frame_makers)
                return (res, ids)
            except Exception:
                # If an error occurs during processing, try displaying the original frame.
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
    finally:
        print("getArucoCode done")
def check_box_qrcodes():
    """Check for the presence of express delivery boxes by detecting ID3 and ID4 QR codes, and identify their stacking relationship and left/right position."""
    global cam
    print("Starting to detect the QR code on the express delivery box...")
    # Check if the camera is turned off or not initialized.
    if not init_camera():
        print("Camera failed!")
        return {"target_id": None, "is_upper": False}
    
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        
        # The number of tests increases the accuracy of the test.
        detect_count = 0
        max_detect_count = 10
        
        id3_detected_times = 0
        id4_detected_times = 0
        
        # Record the (X,Y) coordinates of id3 and id4
        id3_positions = []  # Record the (X,Y) coordinates of id3
        id4_positions = []  # Record the (X,Y) coordinates of id4
        
        # Record all detected markers and their locations for each frame.
        all_markers_data = []
        
        while detect_count < max_detect_count:
            ret, frame = cam.read()
            if not ret or frame is None:
                detect_count += 1
                time.sleep(0.3)
                continue
            
            frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Increase contrast to improve detection success rate
            gray = cv2.equalizeHist(gray)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Save the detection results of the current frame.
            frame_markers = []
            
            # Check if id3 or id4 is detected.
            current_detected_ids = []
            if ids is not None:
                for i, detected_id in enumerate(ids):
                    current_detected_ids.append(detected_id[0])
                    # Calculate the center point of the mark
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
                            id3_detected_times += 1
                            id3_positions.append((center_x, center_y))
                            print(f"id3 was detected ({detect_count+1}th detection), X coordinate: {center_x}, Y coordinate: {center_y}",flush=True)
                        elif detected_id[0] == 4:
                            id4_detected_times += 1
                            id4_positions.append((center_x, center_y))
                            print(f"Detected ID4 ({detect_count+1}th detection), X coordinate: {center_x}, Y coordinate: {center_y}",flush=True)
            
            # Save frame data
            if frame_markers:
                all_markers_data.append(frame_markers)
            
            # Print all IDs detected in the current frame for debugging purposes.
            print(f"ID detected in the {detect_count+1}th iteration:{current_detected_ids}")
            
            detect_count += 1
            time.sleep(0.3)
        
        # Print statistical results
        print(f"Detection complete. id3 was detected {id3_detected_times} times, and id4 was detected {id4_detected_times} times.")
        
        # Set a detection threshold; the system will only consider a virus present if it is detected at least three times.
        min_detection_threshold = 3
        
        # The presence of express delivery boxes is determined based on the number of tests conducted.
        has_id3 = id3_detected_times >= min_detection_threshold
        has_id4 = id4_detected_times >= min_detection_threshold
        
        # Returns a dictionary containing the target ID and whether it is a parent layer.
        result = {"target_id": None, "is_upper": False}
        
        # Count the actual number of express delivery boxes (deduplicated).
        unique_id3_boxes = set()
        unique_id4_boxes = set()
        
        # Determine if they belong to the same box based on the similarity of their X/Y coordinates (allowing for a 10-pixel error).
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
        
        # Convert the collection to a list for easier subsequent processing.
        id3_boxes_list = list(unique_id3_boxes)
        id4_boxes_list = list(unique_id4_boxes)
        
        total_boxes = len(unique_id3_boxes) + len(unique_id4_boxes)
        print(f"In reality, {len(unique_id3_boxes)} id3 boxes and {len(unique_id4_boxes)} id4 boxes were detected, for a total of {total_boxes} boxes.")
        
        # No target was detected
        if total_boxes == 0:
            print("No parcel boxes were detected.")
            return result
        
        # 1. Case with only one express box
        if len(unique_id3_boxes) == 1 and len(unique_id4_boxes) == 0:
            id3_y = id3_boxes_list[0][1]
            # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
            is_upper_id3 = id3_y < 250
            if is_upper_id3:
                print(f"There is only one id3 parcel box, with y-coordinate = {id3_y}, located in the upper area; it can be directly crawled.")
            else:
                print(f"There is only one id3 parcel box, with y-coordinate = {id3_y}, located in the lower area; it can be directly crawled.")
            result["target_id"] = "id3"
            result["is_upper"] = is_upper_id3
            return result
        elif len(unique_id4_boxes) == 1 and len(unique_id3_boxes) == 0:
            id4_y = id4_boxes_list[0][1]
            # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
            is_upper_id4 = id4_y < 250
            if is_upper_id4:
                print(f"There is only one id4 parcel box, with y-coordinate = {id4_y}, located in the upper area; it can be directly crawled.")
            else:
                print(f"There is only one id4 parcel box, with y-coordinate = {id4_y}, located in the lower area; it can be directly crawled.")
            result["target_id"] = "id4"
            result["is_upper"] = is_upper_id4
            return result
        
        # 2. The situation of the two express boxes
        elif total_boxes == 2:
            print("Two express boxes were detected.")
            
            # Case 2.1: One ID3 and one ID4
            if len(unique_id3_boxes) == 1 and len(unique_id4_boxes) == 1:
                print("One ID3 and one ID4")
                
                # Determine the positions of id3 and id4
                id3_x, id3_y = id3_boxes_list[0]
                id4_x, id4_y = id4_boxes_list[0]
                
                #Check for stacking (Y coordinate difference greater than the threshold).
                if abs(id3_y - id4_y) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    if id3_y < id4_y:
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"Stacking was detected; id3 is on the upper layer ({id3_y} < {id4_y}), so the upper-layer id3 will be crawled first.")
                    else:
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        print(f"Stacking was detected; id4 is on the upper layer ({id4_y} < {id3_y}), so id4 on the upper layer will be crawled first.")
                else:
                    #  Without stacking, use a y-coordinate threshold of 250 to determine the upper and lower layers.
                    id3_x, id3_y = id3_boxes_list[0]
                    id4_x, id4_y = id4_boxes_list[0]
                    
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    if id3_y < 250 and id4_y >= 250:
                        # ID3 is in the upper layer, and ID4 is in the lower layer.
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"If different IDs are at the same level, and id3 is in the upper-level region ({id3_y} < 250), then id3 will be crawled first.")
                    elif id4_y < 250 and id3_y >= 250:
                        # ID4 is in the upper layer, and ID3 is in the lower layer.
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        print(f"If different IDs are at the same level, and id4 is in the upper-level region ({id4_y} < 250), then id4 will be crawled first.")
                    else:
                        # If both are in the upper or lower layer, prioritize fetching ID3.
                        result["target_id"] = "id3"
                        # Set the is_upper flag according to the actual region.
                        if id3_y < 250 and id4_y < 250:
                            result["is_upper"] = True
                            print(f"If different IDs are at the same level and all are in the upper area, prioritize crawling ID3.")
                        else:
                            result["is_upper"] = False
                            print(f"If different IDs are at the same level and all are in the lower level area, prioritize crawling ID3.")
                
                print(f"id3 coordinates: ({id3_x}, {id3_y}), id4 coordinates: ({id4_x}, {id4_y})")
                return result
            
            # Scenario 2.2: Two express boxes with the same ID
            elif len(unique_id3_boxes) == 2:
                print("Two ID3 parcel boxes")
                
                # Check for stacking (Y coordinate difference greater than the threshold).
                y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                    print(f"When two ID3s are stacked, the ID3 of the upper layer is crawled first.")
                    result["target_id"] = "id3"
                    result["is_upper"] = True
                    return result
                else:
                    # Without stacking, use a y-coordinate threshold of 250 to determine the upper and lower layers.
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        # One is at the top level, and the other is at the bottom level; the one at the top level will be crawled first.
                        upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                        print(f"If two ID3 values ​​are not stacked, prioritize crawling the ID3 of the upper layer (Y coordinate = {min(y1, y2)} < 250).")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        # One is at the top level, and the other is at the bottom level; the one at the top level will be crawled first.
                        upper_box = id3_boxes_list[1] if y2 < y1 else id3_boxes_list[0]
                        print(f"If two ID3 values ​​are not stacked, prioritize crawling the ID3 of the upper layer (Y coordinate = {min(y1, y2)} < 250).")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    else:
                        # Within the same layer, prioritize capturing the leftmost element (with a smaller X-coordinate).
                        left_box = min(id3_boxes_list, key=lambda pos: pos[0])
                        print(f"If two ID3 values ​​are in the same layer {', upper region' if y1 < 250 else ', lower region'}, prioritize crawling the ID3 value on the left.")
                        result["target_id"] = "id3"
                        # Whether to go to the upper layer is set according to the actual Y-coordinate.
                        result["is_upper"] = y1 < 250
                        return result
            
            elif len(unique_id4_boxes) == 2:
                print("Two ID4 parcel boxes")
                
                # Check for stacking (Y coordinate difference greater than the threshold).
                y1, y2 = id4_boxes_list[0][1], id4_boxes_list[1][1]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    upper_box = id4_boxes_list[0] if y1 < y2 else id4_boxes_list[1]
                    print(f"When two ID4s are stacked, the ID4 of the upper layer is crawled first.")
                    result["target_id"] = "id4"
                    result["is_upper"] = True
                    return result
                else:
                    # Without stacking, use a y-coordinate threshold of 250 to determine the upper and lower layers.
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    y1, y2 = id4_boxes_list[0][1], id4_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        # One is at the top level, and the other is at the bottom level; the one at the top level will be crawled first.
                        upper_box = id4_boxes_list[0] if y1 < y2 else id4_boxes_list[1]
                        print(f"If two ID4 values ​​are not stacked, prioritize crawling the ID4 of the upper layer (Y coordinate = {min(y1, y2)} < 250).")
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        # One is at the top level, and the other is at the bottom level; the one at the top level will be crawled first.
                        upper_box = id4_boxes_list[1] if y2 < y1 else id4_boxes_list[0]
                        print(f"If two ID4 values ​​are not stacked, prioritize crawling the ID4 of the upper layer (Y coordinate = {min(y1, y2)} < 250).")
                        result["target_id"] = "id4"
                        result["is_upper"] = True
                        return result
                    else:
                        # Within the same layer, prioritize capturing the leftmost element (with a smaller X-coordinate).
                        left_box = min(id4_boxes_list, key=lambda pos: pos[0])
                        print(f"If two ID4 values ​​are in the same layer {', upper region' if y1 < 250 else ', lower region'}, prioritize crawling the ID4 value on the left.")
                        result["target_id"] = "id4"
                        # Whether to go to the upper layer is set according to the actual Y-coordinate.
                        result["is_upper"] = y1 < 250
                        return result
        
        # 3. The situation of the three express boxes
        elif total_boxes == 3:
            print("Three express boxes were detected.")
            
            # Collect the location information of all boxes
            all_boxes = []
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))
            for x, y in unique_id4_boxes:
                all_boxes.append((4, x, y))
            
            # Sort by Y-coordinate and find the top-level box (the smaller the Y-value, the more likely it is to be on the top level).
            all_boxes.sort(key=lambda box: box[2])
            
            # There must be one side that is stacked; prioritize grabbing the boxes on the top layer.
            upper_box = all_boxes[0]  # The box with the smallest Y-coordinate
            is_upper = upper_box[2] < 250  # Determine whether it is truly in the upper region based on the actual Y-coordinate.
            print(f"Prioritize grabbing the uppermost parcel box: id{upper_box[0]}, Y coordinate={upper_box[2]}, whether it is an upper region={is_upper}")
            result["target_id"] = f"id{upper_box[0]}"
            result["is_upper"] = is_upper
            return result
        
        # 4. The status of the four parcel boxes (two with ID3 and two with ID4)
        elif total_boxes == 4:
            print("Four parcel boxes were detected (two with ID3 and two with ID4).")
            
            # Collect the location information of all boxes
            all_boxes = []
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))
            for x, y in unique_id4_boxes:
                all_boxes.append((4, x, y))
            
            # Sort by Y-coordinate and find the top-level boxes (the two with smaller Y-coordinates).
            all_boxes.sort(key=lambda box: box[2])
            upper_boxes = all_boxes[:2]  # The two boxes on the top
            
            # Check if all upper-level users have the same ID.
            upper_ids = [box[0] for box in upper_boxes]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"Both upper layers have id{upper_ids[0]}")
                # Prioritize capturing the leftmost (smaller X-coordinate) images.
                upper_boxes.sort(key=lambda box: box[1])
                target_id = f"id{upper_boxes[0][0]}"
                # Determine whether it is truly in the upper region based on the actual Y-coordinate.
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = target_id
                result["is_upper"] = is_upper
                print(f"Prioritize fetching the upper left region {target_id}, and check if it's an upper region = {is_upper}.")
                return result
            else:
                print("The two upper layers have different IDs.")
                # Prioritize crawling ID3
                for box in upper_boxes:
                    if box[0] == 3:
                        print("Prioritize crawling the ID3 of the upper layer.")
                result["target_id"] = "id3"
                # Determine whether it is truly in the upper region based on the actual Y-coordinate.
                result["is_upper"] = box[2] < 250
                print(f"Prioritize fetching the upper-level ID3; whether it's an upper-level region = {result['is_upper']}")
                return result
                # If the upper layer does not have ID3, fetch the first one from the upper layer.
                # Determine whether it is truly in the upper region based on the actual Y-coordinate.
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = f"id{upper_boxes[0][0]}"
                result["is_upper"] = is_upper
                print(f"The upper layer does not have ID3. Retrieve the upper layer's ID {upper_boxes[0][0]}, and check if it's an upper-layer region = {is_upper}.")
                return result
        
        # Default behavior: Return results based on priority.
        if has_id3 and has_id4:
            print("Based on the default priority: prioritize crawling ID3.")
            result["target_id"] = "id3"
            # Calculate the average Y-coordinate to determine the upper and lower layers
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            id4_avg_y = sum(y for x, y in id4_positions) / len(id4_positions) if id4_positions else 0
            result["is_upper"] = id3_avg_y > id4_avg_y
        elif has_id3:
            print("Default: Only ID3 is detected, and ID3 is crawled first.")
            result["target_id"] = "id3"
            # Determine whether it is on the upper layer based on the actual Y-coordinate.
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            result["is_upper"] = id3_avg_y < 250
            print(f"The average Y-coordinate of id3 is set to {id3_avg_y}, and whether it is an upper layer is set to {result['is_upper']}.")
        elif has_id4:
            print("Default: Only id4 is detected, and id4 is crawled first.")
            result["target_id"] = "id4"
            # Determine whether it is on the upper layer based on the actual Y-coordinate.
            id4_avg_y = sum(y for x, y in id4_positions) / len(id4_positions) if id4_positions else 0
            result["is_upper"] = id4_avg_y < 250
            print(f"The average Y-coordinate of id4 is set to {id4_avg_y}, and whether it is an upper layer is set to {result['is_upper']}.")
        
        print(f"Detection results: Target ID = {result['target_id']}, Is it an upper layer = {result['is_upper']}")
        return result
            
    except Exception as e:
        print(f"Error detected QR code: {str(e)}")
        # When an error occurs, id3 is selected by default and assumed to be the upper level.
        return {"target_id": "id3", "is_upper": True}
    finally:
        print("check box qrcodes done")
 
def process_qr_data():
    """
    The system processes QR code data and determines which target's data to return based on the existence of the package and its hierarchical relationship.
    
    Logical rules:
    1. When there is only one package, directly grab the detected package.
    2.When there are two express boxes:
       - Given an ID3 and an ID4: prioritize crawling ID3.
       - Two identical IDs: the one on the left is crawled first; if they are stacked, the one on the top is crawled first.
    3. When there are three packages: one side will inevitably be stacked, so prioritize grabbing the top package.
    4. When there are four boxes: if they are stacked on both sides, prioritize grabbing the top layer; if the same ID is on the top layer, grab the left one; if different IDs are on the top layer, prioritize ID3.
    return:
        If a grabbable parcel box is found: Return (_z, _ry, _perc), where _z is the depth, _ry is the pitch angle, and _perc is the normalized center point value.
        If no parcel can be grabbed: Return -1
    """
    global cam
    if cam is None or not cam.isOpened():
        print("cam not opened")
        return -1
    
    data_ = getArucoCode(True)
    # Check data validity
    if data_ is not None and len(data_) > 1 and data_[1] is not None:
        results, ids = data_
        
        if results == []:
            return -1   #The marker_length of the QR code needs to be large, and the marker_length parameter must be given correctly; otherwise, there will be no pose information.
        
        # Create a mapping from ID to index for easy and fast lookup.
        id_to_index = {}
        for i, marker_id in enumerate(ids):
            id_to_index[marker_id[0]] = i
        
        # Check if a specific ID was detected.
        has_id3 = 3 in id_to_index and id_to_index[3] < len(results)
        has_id4 = 4 in id_to_index and id_to_index[4] < len(results)
        
        # Collect all id3 and id4 tag information
        id3_markers = []
        id4_markers = []
        
        for i, marker_id in enumerate(ids):
            mid = marker_id[0]
            if i < len(results):
                if mid == 3:
                    x, y = results[i][6][0], results[i][6][1]
                    id3_markers.append((i, x, y))  # (Index, X coordinate, Y coordinate)
                elif mid == 4:
                    x, y = results[i][6][0], results[i][6][1]
                    id4_markers.append((i, x, y))  # (Index, X coordinate, Y coordinate)
        
        total_id3 = len(id3_markers)
        total_id4 = len(id4_markers)
        
        print(f"process_qr_data: Detection results: Number of id3s = {total_id3}, Number of id4s = {total_id4}")
        
        # 1. Case with only one express box
        if total_id3 == 1 and total_id4 == 0:
            y_id3 = id3_markers[0][2]
            # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
            if y_id3 < 250:
                print(f"process_qr_data: There is only one id3 parcel box, with y-coordinate = {y_id3}, located in the upper area; it can be directly crawled.")
            else:
                print(f"process_qr_data: There is only one id3 parcel box, with y-coordinate = {y_id3}, located in the lower area, so it can be directly crawled.")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif total_id4 == 1 and total_id3 == 0:
            y_id4 = id4_markers[0][2]
            #The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
            if y_id4 < 250:
                print(f"process_qr_data: There is only one package with ID4, y-coordinate = {y_id4}, located in the upper area; it can be directly captured.")
            else:
                print(f"process_qr_data: There is only one package with ID4, y-coordinate = {y_id4}, located in the lower area; it can be directly captured.")
            i = id4_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # 2. The situation of the two express boxes
        elif (total_id3 + total_id4) == 2:
            print("process_qr_data: Detected 2 express boxes")
            
            # 2.1 One ID3 and one ID4
            if total_id3 == 1 and total_id4 == 1:
                print("process_qr_data: One ID3 and one ID4")
                # Check for stacking (Y coordinate difference greater than the threshold).
                y_id3 = id3_markers[0][2]
                y_id4 = id4_markers[0][2]
                
                if abs(y_id3 - y_id4) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    if y_id3 < y_id4:
                        i = id3_markers[0][0]
                        print("Different IDs are stacked, with id3 on the upper layer, so id3 is crawled first.")
                    else:
                        i = id4_markers[0][0]
                        print("Different IDs are stacked, with id4 at the top, so id4 will be crawled first.")
                else:
                    # Without stacking, use a y-coordinate threshold of 250 to determine the upper and lower layers.
                    y_id3 = id3_markers[0][2]
                    y_id4 = id4_markers[0][2]
                    
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    if y_id3 < 250 and y_id4 >= 250:
                        # ID3 is in the upper layer, and ID4 is in the lower layer.
                        i = id3_markers[0][0]
                        print("If different IDs are in the same layer, and id3 is in the upper layer area, then id3 will be crawled first.")
                    elif y_id4 < 250 and y_id3 >= 250:
                        # ID4 is in the upper layer, and ID3 is in the lower layer.
                        i = id4_markers[0][0]
                        print("Different IDs at the same level, id4 is in the upper level area, so id4 will be crawled first.")
                    else:
                        # If both are in the upper or lower layer, prioritize fetching ID3.
                        i = id3_markers[0][0]
                        # Clearly distinguish areas
                        if y_id3 < 250 and y_id4 < 250:
                            print("If different IDs are at the same level and all are in the upper area, prioritize crawling ID3.")
                        else:
                            print("If different IDs are at the same level and all are in the lower level area, prioritize crawling ID3.")
                
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
            
            # 2.2 Two express boxes with the same ID
            elif total_id3 == 2:
                print("Two ID3 parcel boxes")
                # Check for stacking (Y coordinate difference greater than the threshold).
                y1, y2 = id3_markers[0][2], id3_markers[1][2]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    i = id3_markers[0][0] if y1 < y2 else id3_markers[1][0]
                    print("When two ID3s are stacked, the ID3 of the upper layer is crawled first.")
                else:
                    # Use the y-coordinate threshold of 250 to determine the upper and lower layers.
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    if y1 < 250 and y2 >= 250:
                        # One is at the top level, and the other is at the bottom level; the top level will be crawled first.
                        i = id3_markers[0][0]
                        print("If the two ID3s are not on the same level, prioritize crawling the ID3 of the higher level.")
                    elif y2 < 250 and y1 >= 250:
                        # One is at the top level, and the other is at the bottom level; the top level will be crawled first.
                        i = id3_markers[1][0]
                        print("If the two ID3s are not on the same level, prioritize crawling the ID3 of the higher level.")
                    else:
                        # If both are on the upper layer or both are on the lower layer, prioritize capturing the one on the left (with the smaller X-coordinate).
                        x1, x2 = id3_markers[0][1], id3_markers[1][1]
                        i = id3_markers[0][0] if x1 < x2 else id3_markers[1][0]
                        # Clearly distinguish areas
                        if y1 < 250 and y2 < 250:
                            print("If two ID3 values ​​are on the same level and both are in the upper area, prioritize crawling the ID3 value on the left.")
                        else:
                            print("If two ID3 values ​​are on the same level and both are in the lower area, prioritize crawling the ID3 value on the left.")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
            
            elif total_id4 == 2:
                print("Two ID4 parcel boxes")
                # Check for stacking (Y coordinate difference greater than the threshold).
                y1, y2 = id4_markers[0][2], id4_markers[1][2]
                if abs(y1 - y2) > 20:  # Stacking judgment threshold
                    # If there is stacking, prioritize grabbing the uppermost layer (those with smaller Y-coordinates).
                    i = id4_markers[0][0] if y1 < y2 else id4_markers[1][0]
                    print("When two ID4s are stacked, the one with the upper ID4 will be crawled first.")
                else:
                    # Use the y-coordinate threshold of 250 to determine the upper and lower layers.
                    # The layer with a y-coordinate below 250 is the upper layer, and the layer with a y-coordinate above 250 is the lower layer.
                    if y1 < 250 and y2 >= 250:
                        # One is at the top level, and the other is at the bottom level; the top level will be crawled first.
                        i = id4_markers[0][0]
                        print("If the two ID4s are not on the same level, prioritize crawling the ID4 on the upper level.")
                    elif y2 < 250 and y1 >= 250:
                        # One is at the top level, and the other is at the bottom level; the top level will be crawled first.
                        i = id4_markers[1][0]
                        print("If the two ID4s are not on the same level, prioritize crawling the ID4 on the upper level.")
                    else:
                        # If both are on the upper layer or both are on the lower layer, prioritize capturing the one on the left (with the smaller X-coordinate).
                        x1, x2 = id4_markers[0][1], id4_markers[1][1]
                        i = id4_markers[0][0] if x1 < x2 else id4_markers[1][0]
                        # Clearly distinguish areas
                        if y1 < 250 and y2 < 250:
                            print("If both ID4 values ​​are on the same level and both are in the upper area, prioritize crawling the ID4 value on the left.")
                        else:
                            print("If both ID4 values ​​are on the same level and both are in the lower area, prioritize crawling the ID4 value on the left.")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
        
        # 3. The situation of the three express boxes
        elif (total_id3 + total_id4) == 3:
            print("If three packages were detected, one side must be stacked.")
            
            # Collect the location information of all boxes
            all_markers = []
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))  # (Index, ID, X coordinate, Y coordinate)
            for i, x, y in id4_markers:
                all_markers.append((i, 4, x, y))
            
            # Sort by Y-coordinate and find the top-level box (the smaller the Y-value, the more likely it is to be on the top level).
            all_markers.sort(key=lambda m: m[3])
            
            # Prioritize grabbing the upper-layer parcel boxes
            upper_marker = all_markers[0]
            i = upper_marker[0]
            print(f"Prioritize fetching the upper-level parcel boxes: id{upper_marker[1]}")
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # 4. The status of the four parcel boxes (two with ID3 and two with ID4)
        elif (total_id3 + total_id4) == 4 and total_id3 == 2 and total_id4 == 2:
            print("Four parcel boxes were detected, stacked on both sides.")
            
            # Collect the location information of all boxes
            all_markers = []
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))  # (index, ID, X coordinate, Y coordinate)
            for i, x, y in id4_markers:
                all_markers.append((i, 4, x, y))
            
            # Sort by Y-coordinate and find the top-level boxes (the two with smaller Y-coordinates).
            all_markers.sort(key=lambda m: m[3])
            upper_markers = all_markers[:2]  # The two boxes on the top
            
            # Check if all upper-level users have the same ID.
            upper_ids = [m[1] for m in upper_markers]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"Both upper layers have id{upper_ids[0]}")
                # Prioritize capturing the leftmost (smaller X-coordinate) images.
                upper_markers.sort(key=lambda m: m[2])
                target_marker = upper_markers[0]
                i = target_marker[0]
                print(f"Prioritize crawling the top left-hand top-level ID {target_marker[1]}")
            else:
                print("The two upper layers have different IDs.")
                # Prioritize crawling ID3
                target_marker = None
                for m in upper_markers:
                    if m[1] == 3:
                        target_marker = m
                        break
                
                if target_marker:
                    i = target_marker[0]
                    print("Prioritize crawling the ID3 of the upper layer.")
                else:
                    # If the upper layer does not have ID3, fetch the first one from the upper layer.
                    i = upper_markers[0][0]
                    print(f"The upper layer does not have id3, so retrieve the upper layer's id.{upper_markers[0][1]}")
            
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # Default: Return results based on priority
        if has_id3:
            print("Default priority: Fetch ID3 first")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif has_id4:
            print("Default priority: Fetch ID4 first")
            i = id4_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        
        # If no parcel can be grabbed, return -1
        print("No grabbable parcel boxes were detected.")
        return -1
    
    # No markers detected
    print("No markers detected")
    return -1

def process_qr_data_2():

    
    data_ = getArucoCode(True)
    # print("data_",data_)
    # example：data_ ([[13.190542591653859, 0.6577785493956305, 30.57489950772101, 56.677235927555834, -10.703176777425517, -17.524720968623765, (892, 292)]], array([[2]], dtype=int32))
    
    if data_ is not None:
        if data_[0] == []:
            return -1   #The marker_length of the QR code needs to be large, and the marker_length parameter must be given correctly; otherwise, there will be no pose information.

        _z = data_[0][0][2]
        _ry = data_[0][0][4]
        _perc = data_[0][0][6][0]/960.0 # Normalized [0,1], (892, 292) are the coordinates of the center point of the QR code.
        return (_z, _ry, _perc) # Returns the center point of depth z, pitch, and screen resolution (960).
    else:
        return -1
def process_qr_data_simple():

    data_ = getArucoCode(True)
    
    if data_ is None or len(data_) < 2:
        return -1
    
    results, ids = data_
    if not results or ids is None:
        return -1

    valid_markers = []
    for i, marker_id in enumerate(ids):
        mid = marker_id[0]
        if mid in [3, 4] and i < len(results):
            valid_markers.append({
                'id': mid,
                'x': results[i][6][0],
                'z': results[i][2],
                'ry': results[i][4],
                'index': i
            })

    if not valid_markers:
        return -1

    # This sorts ID 3 to the beginning. 
    # If there are multiple ID 3s, it sorts the left-most one (smallest X) to the front.
    valid_markers.sort(key=lambda b: (b['id'] != 3, b['x']))

    # 3. Always take the first one after sorting
    target = valid_markers[0]
    
    print(f"Target selected: ID {target['id']} at X={target['x']:.1f}")

    # 4. Return the data
    _z = target['z']
    _ry = target['ry']
    _perc = target['x'] / 960.0 
    
    return (_z, _ry, _perc)
if __name__=='__main__':
    pass
