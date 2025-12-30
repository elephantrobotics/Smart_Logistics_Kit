import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math
from collections import deque

# Camera internals: intrinsic parameter matrix of the camera
camera_matrix = np.array([[785.855437,  0.000000,   451.670922], 
                          [0.000000,    584.820336, 259.056856],
                          [0.000000,    0.000000,   1.000000]])

# Distortion coefficient matrix
dist_matrix = np.array(([[0.095135, -0.109279, -0.002513,  -0.002418, 0.000000]]))

DEBUG = False

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

cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
distance_values = deque(maxlen=5)

while True:
    ret, frame = cap.read()
    # height, width = frame.shape[:2]
    # print(f"Height: {height}, Width: {width}")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters() # Initialize detector parameters with default values

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

    if ids is not None:
        # Use different size parameters based on QR code ID
        if ids is not None and len(ids) > 0:
            # For id3 and id4 use 30mm (0.030 meters), others use 40mm (0.040 meters)
            current_marker_length = 0.030 if ids[0] in [3, 4] else 0.040
        else:
            current_marker_length = 0.040
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, current_marker_length, camera_matrix, dist_matrix)

        (rvec-tvec).any() # get rid of that nasty numpy value array error

        for i in range(rvec.shape[0]):
            # Use the current QR code's corresponding size parameter
            cv2.drawFrameAxes(frame, camera_matrix, dist_matrix, rvec[i, :, :], tvec[i, :, :], current_marker_length)
            cv2.aruco.drawDetectedMarkers(frame, corners,ids)

        cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        distance = ((tvec[0][0][2]) * 100) - 5
        cv2.putText(frame, 'distance:' + str(round(distance, 4)) + str('cm'), (0, 110), font, 1, (0, 255, 0), 2,cv2.LINE_AA)

        distance_values.append(distance)
        if DEBUG == True:
            if len(distance_values)>=5:
                average_distance=np.mean(distance_values)
                cv2.putText(frame, 'distance:' + str(round(average_distance, 4)) + str('cm'), (0, 110), font, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)
                distance_values = []

    else:
        ##### DRAW "NO IDS" #####
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    cv2.imshow("frame",frame)

    key = cv2.waitKey(1 )

    if key == 27:        
        print('esc break...')
        cap.release()
        cv2.destroyAllWindows()
        break
