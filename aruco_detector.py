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


font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)

CAM_WIDTH = 960
CAM_HEIGHT = 540
CAM_FPS = 21

center = (CAM_WIDTH / 2, CAM_HEIGHT / 2)
focal_length = CAM_WIDTH

camera_matrix = np.array([[785.855437,  0.000000,   451.670922], 
                          [0.000000,    584.820336, 259.056856],
                          [0.000000,    0.000000,   1.000000]])

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
                    current_marker_length = 0.03 if ids[0] in [2, 3] else 0.04
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, current_marker_length, camera_matrix, dist_coeffs)
                    for i in range(rvec.shape[0]):
                        imgWithAruco = cv2.drawFrameAxes(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, current_marker_length)

                    # --- The midpoint displays the ID number
                    cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                                 int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

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

                    pose_data_dict[str(ids)] = pose_data.copy()

                    roll_deg = math.degrees(roll_marker)
                    pitch_deg = math.degrees(pitch_marker)
                    yaw_deg = math.degrees(yaw_marker)

                    # print(f"DEBUG _detect: ID={ids[0]}, yaw={yaw_deg:.1f}, pitch={pitch_deg:.1f}, roll={roll_deg:.1f}, dist_to_90={min(abs(yaw_deg) % 90.0, 90.0 - (abs(yaw_deg) % 90.0)):.1f}")
                    _remainder = abs(yaw_deg) % 90.0
                    _dist_to_nearest_90 = min(_remainder, 90.0 - _remainder)
                    if _dist_to_nearest_90 < 30:
                        return [tvec[0] * 100, tvec[1] * 100, tvec[2] * 100, roll_deg, pitch_deg, yaw_deg, cornerMid]
                    # else:
                    #     print(f"DEBUG _detect: ID={ids[0]} REJECTED by yaw filter (dist_to_90={_dist_to_nearest_90:.1f} >= 30)")
                except Exception as e:
                    # print(f"DEBUG _detect: ID={ids}, EXCEPTION: {e}")
                    pass
        except Exception:
            pass

        pose_data[0] = None
        pose_data[1] = None
        pose_data[2] = None
        pose_data[3] = None
        pose_data_dict[0] = pose_data.copy()
        return None

def displayFrame(frame_input):
    try:
        if not cv2.getWindowProperty("show", cv2.WND_PROP_VISIBLE):
            cv2.namedWindow("show", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("show", frame_input)
        cv2.waitKey(1)
    except Exception:
        pass
def closeDisplayFrame():
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
def getArucoCode(display_mode = True ):
    global cam
    if not init_camera():
        print("Camera failed!")
        return None
    
    # read frame once
    frame_makers = None
    try:
        ret, frame = cam.read() 
        if not ret or frame is None:
            print("get aruco code none",ret,cam)
            return None
        
        frame = cv2.flip(frame,-1) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 
        
        parameters = cv2.aruco.DetectorParameters() 
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) 

        if ids is not None:
            try:
                frame_makers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                ids_len = len(ids)

                res = []
                if ids_len > 0:
                    for i in range(ids_len):
                        try:
                            aruco_res = _detect(corners[i:i+1], ids[i], frame) 
                            if aruco_res != None:
                                res.append(aruco_res)
                        except Exception:
                            pass  
                    if display_mode and frame_makers is not None:
                        displayFrame(frame_makers)
                # print(f"DEBUG getArucoCode: detected_ids={ids}, passed_detect={len(res)}")
                return (res, ids)
            except Exception:
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
    """Check for delivery boxes by detecting the id2 and id3 QR codes, and identify the stacking relationship and left-right positions"""
    global cam
    print("Start checking box QR codes...")
    if not init_camera():
        print("Camera failed!")
        return {"target_id": None, "is_upper": False}
    
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        
        detect_count = 0
        max_detect_count = 10
        
        id2_detected_times = 0
        id3_detected_times = 0

        id2_positions = [] 
        id3_positions = []  

        all_markers_data = []
        
        while detect_count < max_detect_count:
            ret, frame = cam.read()
            if not ret or frame is None:
                detect_count += 1
                time.sleep(0.3)
                continue
            
            frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            gray = cv2.equalizeHist(gray)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            frame_markers = []

            current_detected_ids = []
            if ids is not None:
                for i, detected_id in enumerate(ids):
                    current_detected_ids.append(detected_id[0])
                    if corners[i] is not None and len(corners[i]) > 0:
                        corner = corners[i][0]
                        center_x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
                        center_y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4

                        frame_markers.append({
                            "id": detected_id[0],
                            "x": center_x,
                            "y": center_y
                        })
                        
                        if detected_id[0] == 2:
                            id2_detected_times += 1
                            id2_positions.append((center_x, center_y))
                            print(f"ID2 detected (The{detect_count+1}th detection)，X coordinate: {center_x}，Y coordinate: {center_y}",flush=True)
                        elif detected_id[0] == 3:
                            id3_detected_times += 1
                            id3_positions.append((center_x, center_y))
                            print(f"ID3 detected (The{detect_count+1}th detection)，X coordinate: {center_x}，Y coordinate: {center_y}",flush=True)
            
            # Save frame data
            if frame_markers:
                all_markers_data.append(frame_markers)
            
            # current frame detected IDs
            print(f"Frame {detect_count+1}: {current_detected_ids}")
            
            detect_count += 1
            time.sleep(0.3)
        
        # Detection results
        print(f"Detection completed. id2 detected {id2_detected_times} times, id3 detected {id3_detected_times} times")
        
        # Set detection threshold to 3 times
        min_detection_threshold = 3
        
        # Check if delivery boxes are detected by id2 and id3 QR codes
        has_id2 = id2_detected_times >= min_detection_threshold
        has_id3 = id3_detected_times >= min_detection_threshold

        result = {"target_id": None, "is_upper": False}
        
        # Count unique delivery boxes
        unique_id2_boxes = set()
        unique_id3_boxes = set()
        
        # Check if the boxes are the same based on X/Y coordinates (allow 10 pixel error)
        for x, y in id2_positions:
            found = False
            for box_x, box_y in list(unique_id2_boxes):
                if abs(x - box_x) < 10 and abs(y - box_y) < 10:
                    found = True
                    break
            if not found:
                unique_id2_boxes.add((x, y))
        
        for x, y in id3_positions:
            found = False
            for box_x, box_y in list(unique_id3_boxes):
                if abs(x - box_x) < 10 and abs(y - box_y) < 10:
                    found = True
                    break
            if not found:
                unique_id3_boxes.add((x, y))
        
        # Convert set to list for further processing
        id2_boxes_list = list(unique_id2_boxes)
        id3_boxes_list = list(unique_id3_boxes)
        
        total_boxes = len(unique_id2_boxes) + len(unique_id3_boxes)
        print(f"actually detected {len(unique_id2_boxes)} id2 boxes and {len(unique_id3_boxes)} id3 boxes")
        
        # Check if any delivery box is detected
        if total_boxes == 0:
            print("No delivery box detected")
            return result
        
        # 1. one delivery box case
        if len(unique_id2_boxes) == 1 and len(unique_id3_boxes) == 0:
            id2_y = id2_boxes_list[0][1]
            # y coordinate -<250 is upper, 250 is
            is_upper_id2 = id2_y < 250
            if is_upper_id2:
                print(f"only one id2 box detected, y coordinate={id2_y}，In the upper area, grab directly")
            else:
                print(f"only one id2 box detected, y coordinate={id2_y}，In the lower area, grab directly")
            result["target_id"] = "id2"
            result["is_upper"] = is_upper_id2
            return result
        elif len(unique_id3_boxes) == 1 and len(unique_id2_boxes) == 0:
            id3_y = id3_boxes_list[0][1]
            # y coordinate -<250 is upper, 250 is lower
            is_upper_id3 = id3_y < 250
            if is_upper_id3:
                print(f"only one id3 box detected, y coordinate={id3_y}，In the upper area, grab directly")
            else:
                print(f"only one id3 box detected, y coordinate={id3_y}，In the lower area, grab directly")
            result["target_id"] = "id3"
            result["is_upper"] = is_upper_id3
            return result
        
        # 2. two delivery boxes case
        elif total_boxes == 2:
            print("detect 2 delivery boxes")
            
            # 2.1: one id2 and one id3 case
            if len(unique_id2_boxes) == 1 and len(unique_id3_boxes) == 1:
                print("one id2 and one id3")

                id2_x, id2_y = id2_boxes_list[0]
                id3_x, id3_y = id3_boxes_list[0]

                if abs(id2_y - id3_y) > 20: 
                    if id2_y < id3_y:
                        result["target_id"] = "id2"
                        result["is_upper"] = True
                        print(f"overlap detected, id2 is upper ({id2_y} < {id3_y})，grab id2")
                    else:
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"overlap detected, id3 is upper ({id3_y} < {id2_y})，grab id3")
                else:
                    id2_x, id2_y = id2_boxes_list[0]
                    id3_x, id3_y = id3_boxes_list[0]
                    if id2_y < 250 and id3_y >= 250:
                        result["target_id"] = "id2"
                        result["is_upper"] = True
                        print(f"id2 in upper area ({id2_y} < 250)，grab id2")
                    elif id3_y < 250 and id2_y >= 250:
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        print(f"id3 in upper area ({id3_y} < 250)，grab id3")
                    else:
                        result["target_id"] = "id2"
                        if id2_y < 250 and id3_y < 250:
                            result["is_upper"] = True
                            print(f"Different IDs on the same layer and both in the upper area, prioritize capturing id2")
                        else:
                            result["is_upper"] = False
                            print(f"Different IDs on the same layer and both in the lower area, prioritize capturing id2")
                
                print(f"id2 position: ({id2_x}, {id2_y}), id3 position: ({id3_x}, {id3_y})")
                return result

            elif len(unique_id2_boxes) == 2:
                print("two id2 boxes")

                y1, y2 = id2_boxes_list[0][1], id2_boxes_list[1][1]
                if abs(y1 - y2) > 20: 
                    upper_box = id2_boxes_list[0] if y1 < y2 else id2_boxes_list[1]
                    print(f"Two id2 stacked, prioritize grabbing the upper id2")
                    result["target_id"] = "id2"
                    result["is_upper"] = True
                    return result
                else:
                    y1, y2 = id2_boxes_list[0][1], id2_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        upper_box = id2_boxes_list[0] if y1 < y2 else id2_boxes_list[1]
                        print(f"Two id2 stacked, prioritize grabbing the upper id2 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id2"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        upper_box = id2_boxes_list[1] if y2 < y1 else id2_boxes_list[0]
                        print(f"Two id2 stacked, prioritize grabbing the upper id2 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id2"
                        result["is_upper"] = True
                        return result
                    else:
                        left_box = min(id2_boxes_list, key=lambda pos: pos[0])
                        print(f"Two id2 on the same layer{' in upper area' if y1 < 250 else ', left area'}")
                        result["target_id"] = "id2"
                        result["is_upper"] = y1 < 250
                        return result
            
            elif len(unique_id3_boxes) == 2:
                print("two id3 boxes")

                y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                if abs(y1 - y2) > 20: 
                    upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                    print(f"Two id3 stacked, prioritize grabbing the upper id3")
                    result["target_id"] = "id3"
                    result["is_upper"] = True
                    return result
                else:
                    y1, y2 = id3_boxes_list[0][1], id3_boxes_list[1][1]
                    
                    if y1 < 250 and y2 >= 250:
                        upper_box = id3_boxes_list[0] if y1 < y2 else id3_boxes_list[1]
                        print(f"Two id3 stacked, prioritize grabbing the upper id3 (Y coordinate={min(y1, y2)} < 250)")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    elif y2 < 250 and y1 >= 250:
                        upper_box = id3_boxes_list[1] if y2 < y1 else id3_boxes_list[0]
                        print(f"Two id3 stacked, prioritize grabbing the upper id3 (Y坐标={min(y1, y2)} < 250)")
                        result["target_id"] = "id3"
                        result["is_upper"] = True
                        return result
                    else:
                        left_box = min(id3_boxes_list, key=lambda pos: pos[0])
                        print(f"Two id3 on the same layer{' in upper area' if y1 < 250 else ', left area'}")
                        result["target_id"] = "id3"
                        result["is_upper"] = y1 < 250
                        return result

        elif total_boxes == 3:
            print("Three boxes")

            all_boxes = []
            for x, y in unique_id2_boxes:
                all_boxes.append((2, x, y))
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))

            all_boxes.sort(key=lambda box: box[2])

            upper_box = all_boxes[0] 
            is_upper = upper_box[2] < 250  
            print(f"Prioritize picking up the packages on the top layer: id{upper_box[0]}，Y coordinate={upper_box[2]}，is_upper={is_upper}")
            result["target_id"] = f"id{upper_box[0]}"
            result["is_upper"] = is_upper
            return result

        elif total_boxes == 4:
            print("Detected 4 courier boxes (two id2 and two id3)")

            all_boxes = []
            for x, y in unique_id2_boxes:
                all_boxes.append((2, x, y))
            for x, y in unique_id3_boxes:
                all_boxes.append((3, x, y))

            all_boxes.sort(key=lambda box: box[2])
            upper_boxes = all_boxes[:2] 

            upper_ids = [box[0] for box in upper_boxes]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"The upper layer on both sides is id{upper_ids[0]}")
                upper_boxes.sort(key=lambda box: box[1])
                target_id = f"id{upper_boxes[0][0]}"
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = target_id
                result["is_upper"] = is_upper
                print(f"Prioritize grabbing the left upper layer: {target_id}，is_upper={is_upper}")
                return result
            else:
                print("The upper layer on both sides is different ID")
                for box in upper_boxes:
                    if box[0] == 2:
                        print("Pick up the upper id2")
                result["target_id"] = "id2"
                result["is_upper"] = box[2] < 250
                print(f"Prioritize capturing the upper-level id2, whether it is the upper-level area={result['is_upper']}")
                return result
                is_upper = upper_boxes[0][2] < 250
                result["target_id"] = f"id{upper_boxes[0][0]}"
                result["is_upper"] = is_upper
                print(f"The upper level does not have an id2, capture the id from the upper level: id{upper_boxes[0][0]}，is_upper={is_upper}")
                return result

        if has_id2 and has_id3:
            print("Prioritize grabbing the upper-level id2")
            result["target_id"] = "id2"
            id2_avg_y = sum(y for x, y in id2_positions) / len(id2_positions) if id2_positions else 0
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            result["is_upper"] = id2_avg_y > id3_avg_y
        elif has_id2:
            print("Prioritize grabbing the upper-level id2")
            result["target_id"] = "id2"

            id2_avg_y = sum(y for x, y in id2_positions) / len(id2_positions) if id2_positions else 0
            result["is_upper"] = id2_avg_y < 250
            print(f"id2 average Y coordinate={id2_avg_y}，Is it upper?={result['is_upper']}")
        elif has_id3:
            print("Prioritize grabbing the upper-level id3")
            result["target_id"] = "id3"
            id3_avg_y = sum(y for x, y in id3_positions) / len(id3_positions) if id3_positions else 0
            result["is_upper"] = id3_avg_y < 250
            print(f"id3 average Y coordinate={id3_avg_y}，Is it upper?={result['is_upper']}")
        
        print(f"Detection result: target ID={result['target_id']}, is_upper={result['is_upper']}")
        return result
            
    except Exception as e:
        print(f"Error detecting QR code: {str(e)}")
        return {"target_id": "id2", "is_upper": True}
    finally:
        close_camera()
        print("check box qrcodes done")
 
def process_qr_data():
    """
    Process QR code data and determine which target's data to return based on the existence of the delivery boxes and their upper-lower relationships.

    Logic rules:
    1. When there is only one delivery box, directly grab the detected box.
    2. When there are two delivery boxes:
    - One is id2 and one is id3: prioritize grabbing id2.
    - Two boxes with the same ID: prioritize grabbing the left one; if stacked, prioritize grabbing the upper one.
    3. When there are three delivery boxes: one side must be stacked, prioritize grabbing the upper box.
    4. When there are four delivery boxes: both sides are stacked, prioritize grabbing the upper one; if the same ID, grab the left upper one; if different IDs, prioritize id2 on the upper side.

    Return:
    If a grabbable delivery box is found: return (_z, _ry, _perc), where _z is depth, _ry is pitch angle, and _perc is the normalized center value.
    If no grabbable delivery box is found: return -1.
    """
    global cam
    if cam is None or not cam.isOpened():
        print("cam not opened")
        return -1
    
    data_ = getArucoCode(True)
    if data_ is not None and len(data_) > 1 and data_[1] is not None:
        results, ids = data_
        
        if results == []:
            return -1  

        id_to_index = {}
        for i, marker_id in enumerate(ids):
            id_to_index[marker_id[0]] = i

        has_id2 = 2 in id_to_index and id_to_index[2] < len(results)
        has_id3 = 3 in id_to_index and id_to_index[3] < len(results)

        id2_markers = []
        id3_markers = []
        
        for i, marker_id in enumerate(ids):
            mid = marker_id[0]
            if i < len(results):
                if mid == 2:
                    x, y = results[i][6][0], results[i][6][1]
                    id2_markers.append((i, x, y)) 
                elif mid == 3:
                    x, y = results[i][6][0], results[i][6][1]
                    id3_markers.append((i, x, y))  
        
        total_id2 = len(id2_markers)
        total_id3 = len(id3_markers)
        
        print(f"process_qr_data: Test Result: Number of id2={total_id2}, Number of id3={total_id3}")
        
        if total_id2 == 1 and total_id3 == 0:
            y_id2 = id2_markers[0][2]
            if y_id2 < 250:
                print(f"process_qr_data: Only one id2 express box, y-coordinate={y_id2}，In the upper area, grab directly")
            else:
                print(f"process_qr_data: Only one id2 express box, y-coordinate={y_id2}，In the lower area, grab directly")
            i = id2_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif total_id3 == 1 and total_id2 == 0:
            y_id3 = id3_markers[0][2]
            if y_id3 < 250:
                print(f"process_qr_data: Only one id3 express box, y-coordinate={y_id3}，In the upper area, grab directly")
            else:
                print(f"process_qr_data: Only one id3 express box, y-coordinate={y_id3}，In the lower area, grab directly")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)

        elif (total_id2 + total_id3) == 2:
            print("process_qr_data: Detect 2 delivery boxes")

            if total_id2 == 1 and total_id3 == 1:
                print("process_qr_data: One id2 and one id3")
                y_id2 = id2_markers[0][2]
                y_id3 = id3_markers[0][2]
                
                if abs(y_id2 - y_id3) > 20: 
                    if y_id2 < y_id3:
                        i = id2_markers[0][0]
                        print("process_qr_data: Different ID stacked, prioritize grabbing id2")
                    else:
                        i = id3_markers[0][0]
                        print("process_qr_data: Different ID stacked, prioritize grabbing id3 express box")
                else:
                    y_id2 = id2_markers[0][2]
                    y_id3 = id3_markers[0][2]

                    if y_id2 < 250 and y_id3 >= 250:
                        i = id2_markers[0][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                    elif y_id3 < 250 and y_id2 >= 250:
                        i = id3_markers[0][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id3 express box")
                    else:
                        i = id2_markers[0][0]
                        if y_id2 < 250 and y_id3 < 250:
                            print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                        else:
                            print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)

            elif total_id2 == 2:
                print("Two ID2 express boxes")
                y1, y2 = id2_markers[0][2], id2_markers[1][2]
                if abs(y1 - y2) > 20: 
                    i = id2_markers[0][0] if y1 < y2 else id2_markers[1][0]
                    print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                else:
                    if y1 < 250 and y2 >= 250:
                        i = id2_markers[0][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                    elif y2 < 250 and y1 >= 250:
                        i = id2_markers[1][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id2")
                    else:
                        x1, x2 = id2_markers[0][1], id2_markers[1][1]
                        i = id2_markers[0][0] if x1 < x2 else id2_markers[1][0]
                        if y1 < 250 and y2 < 250:
                            print("When two id2s are on the same level and both in the upper area, the left id2 is prioritized.")
                        else:
                            print("When two id2s are on the same level and both in the lower area, the left id2 is prioritized.")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)
            
            elif total_id3 == 2:
                print("Two ID3 express boxes")
                y1, y2 = id3_markers[0][2], id3_markers[1][2]
                if abs(y1 - y2) > 20:  
                    i = id3_markers[0][0] if y1 < y2 else id3_markers[1][0]
                    print("process_qr_data: Same ID stacked, prioritize grabbing id3 express box")
                else:
                    if y1 < 250 and y2 >= 250:
                        i = id3_markers[0][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id3 express box")
                    elif y2 < 250 and y1 >= 250:
                        i = id3_markers[1][0]
                        print("process_qr_data: Same ID stacked, prioritize grabbing id3 express box")
                        x1, x2 = id3_markers[0][1], id3_markers[1][1]
                        i = id3_markers[0][0] if x1 < x2 else id3_markers[1][0]
                        if y1 < 250 and y2 < 250:
                            print("When two id3s are on the same level and both in the upper area, the left id3 express box is prioritized.")
                        else:
                            print("When two id3s are on the same level and both in the lower area, the left id3 express box is prioritized.")
                _z = results[i][2]
                _ry = results[i][4]
                _perc = results[i][6][0]/960.0
                return (_z, _ry, _perc)

        elif (total_id2 + total_id3) == 3:
            print("Three delivery boxes were detected, and one side must be stacked.")
            
            all_markers = []
            for i, x, y in id2_markers:
                all_markers.append((i, 2, x, y)) 
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))

            all_markers.sort(key=lambda m: m[3])

            upper_marker = all_markers[0]
            i = upper_marker[0]
            print(f"Prioritize picking up the top-layer courier boxes: id{upper_marker[1]}")
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)

        elif (total_id2 + total_id3) == 4 and total_id2 == 2 and total_id3 == 2:
            print("Four delivery boxes were detected, and both sides must be stacked.")

            all_markers = []
            for i, x, y in id2_markers:
                all_markers.append((i, 2, x, y)) 
            for i, x, y in id3_markers:
                all_markers.append((i, 3, x, y))

            all_markers.sort(key=lambda m: m[3])
            upper_markers = all_markers[:2]  

            upper_ids = [m[1] for m in upper_markers]
            
            if upper_ids[0] == upper_ids[1]:
                print(f"Both sides are upper layer id{upper_ids[0]}")
                upper_markers.sort(key=lambda m: m[2])
                target_marker = upper_markers[0]
                i = target_marker[0]
                print(f"Prioritize picking up the left upper-layer courier box: id{target_marker[1]}")
            else:
                print("Both sides are upper layer different ID")
                target_marker = None
                for m in upper_markers:
                    if m[1] == 2:
                        target_marker = m
                        break
                
                if target_marker:
                    i = target_marker[0]
                    print("Prioritize grabbing the upper-level id2")
                else:
                    i = upper_markers[0][0]
                    print(f"The upper layer does not have an id2, capture the id from the upper layer{upper_markers[0][1]}")
            
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)

        if has_id2:
            print("Prioritize grabbing the upper-level id2")
            i = id2_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)
        elif has_id3:
            print("Prioritize grabbing the upper-level id3")
            i = id3_markers[0][0]
            _z = results[i][2]
            _ry = results[i][4]
            _perc = results[i][6][0]/960.0
            return (_z, _ry, _perc)

        print("No valid delivery boxes detected.")
        return -1
    
    print("No markers detected.")
    return -1

def process_qr_data_2():

    
    data_ = getArucoCode(True)
    # print("data_",data_)
    # data_ ([[13.190542591653859, 0.6577785493956305, 30.57489950772101, 56.677235927555834, -10.703176777425517, -17.524720968623765, (892, 292)]], array([[2]], dtype=int32))
    
    if data_ is not None:
        if data_[0] == []:
            return -1   

        _z = data_[0][0][2]
        _ry = data_[0][0][4]
        _perc = data_[0][0][6][0]/960.0 
        return (_z, _ry, _perc) 
    else:
        return -1
def process_qr_data_simple():

    data_ = getArucoCode(True)
    
    if data_ is None or len(data_) < 2:
        # print("DEBUG process_qr_data_simple: getArucoCode returned None/empty")
        return -1
    
    results, ids = data_
    # print(f"DEBUG process_qr_data_simple: results_count={len(results)}, ids={ids}")
    if not results or ids is None:
        # print("DEBUG process_qr_data_simple: results empty or ids is None")
        return -1

    valid_markers = []
    for i, marker_id in enumerate(ids):
        mid = marker_id[0]
        if mid in [2, 3] and i < len(results):
            valid_markers.append({
                'id': mid,
                'x': results[i][6][0],
                'z': results[i][2],
                'ry': results[i][4],
                'index': i
            })

    if not valid_markers:
        return -1

    # This sorts ID 2 to the beginning. 
    # If there are multiple ID 2s, it sorts the left-most one (smallest X) to the front.
    valid_markers.sort(key=lambda b: (b['id'] != 2, b['x']))

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