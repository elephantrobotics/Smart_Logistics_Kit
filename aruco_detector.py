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

cam = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
ret, frame = cam.read()
frame = cv2.flip(frame,-1)

width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
count = cam.get(cv2.CAP_PROP_FRAME_COUNT)
fps = cam.get(cv2.CAP_PROP_FPS)

print(f"Width: {width}, Height: {height}, Count: {count}, FPS: {fps}") # 960,960,-1,21

size = frame.shape
focal_length = size[1]
center = (size[1] / 2, size[0] / 2)

# Camera internals 摄像头的内部参数矩阵
camera_matrix = np.array([[focal_length, 0, center[0]], 
                          [0, focal_length, center[1]],
                          [0, 0, 1]], 
                          dtype="double")

# 畸变系数矩阵
dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,  6.70503562e-03, 2.57018000e+00]]))

print(camera_matrix,dist_coeffs)

cv2.namedWindow("show",cv2.WINDOW_AUTOSIZE)

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
        if len(corners) > 0:
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
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix,dist_coeffs)
                for i in range(rvec.shape[0]):
                    imgWithAruco = cv2.drawFrameAxes(imgWithAruco, camera_matrix,dist_coeffs, rvec, tvec,marker_length)

                    frame_makers =   aruco.drawDetectedMarkers(imgWithAruco.copy(),corners)

                # --- The midpoint displays the ID number
                cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                             int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                cv2.putText(frame, "id=" + str(ids), cornerMid,
                            font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                rvec = rvec[0][0]
                tvec = tvec[0][0]
                # --- Print the tag position in camera frame
                str_position = "MARKER Position x=%.4f (cm)  y=%.4f (cm)  z=%.4f (cm)" % (
                    tvec[0] * 100, tvec[1] * 100, tvec[2] * 100)
                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)
                # -- Print the marker's attitude respect to camera frame
                str_attitude = "MARKER Attitude degrees r=%.4f  p=%.4f  y=%.4f" % (
                    math.degrees(roll_marker), math.degrees(pitch_marker),
                    math.degrees(yaw_marker))

                pose_data[0] = tvec[0] * 100
                pose_data[1] = tvec[1] * 100
                pose_data[2] = tvec[2] * 100
                pose_data[3] = math.degrees(roll_marker)
                pose_data[4] = math.degrees(pitch_marker)
                pose_data[5] = math.degrees(yaw_marker)

                pose_data_dict[ids] = pose_data

                roll_deg = math.degrees(roll_marker)
                pitch_deg = math.degrees(pitch_marker)
                yaw_deg = math.degrees(yaw_marker)

                if abs(yaw_deg)%90.0 < 30:
                    return [tvec[0] * 100, tvec[1] * 100, tvec[2] * 100 , roll_deg, pitch_deg ,yaw_deg, cornerMid]
                else:
                    return None

        else:
            pose_data[0] = None
            pose_data[1] = None
            pose_data[2] = None
            pose_data[3] = None

            pose_data_dict[0] = pose_data
            return None

def displayFrame(frame_input):
    cv2.imshow("show", frame_input)
    cv2.waitKey(1)

def getArucoCode(display_mode = True ):
    #while True: 
        # read frame once
    frame_makers = None
    ret, frame = cam.read() #获取相机的数据流
    frame = cv2.flip(frame,-1) #垂直镜像翻转
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #灰度化
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250) #设置预定义的字典
    
    parameters = cv2.aruco.DetectorParameters() #使用默认值初始化检测器参数
    
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标

    if ids is not None:
        frame_makers = aruco.drawDetectedMarkers(frame.copy(),corners,ids)
        ids_len  = len(ids)

        res = []
        i =0
        if ids_len > 0:
            for l in range(ids_len):
                aruco_res = _detect(corners[i:i+1], ids[i][0], frame) #输入四个角点坐标，计算并返回x,y,z (units is cm), roll, pitch, yaw (units is degree),Aruco id
                if aruco_res != None:
                    res.append(aruco_res)
                i+=1
                if display_mode :
                    displayFrame(frame_makers)
        return (res, ids)

    else:
        # no data detected
        frame_makers = frame.copy()
        if display_mode :
            displayFrame(frame_makers)
        return None
    
    
def process_qr_data():
    data_ = getArucoCode(True)
    # print("data_",data_)
    # 例子：data_ ([[13.190542591653859, 0.6577785493956305, 30.57489950772101, 56.677235927555834, -10.703176777425517, -17.524720968623765, (892, 292)]], array([[2]], dtype=int32))
    
    if data_ is not None:
        if data_[0] == []:
            print("can't not dectet pose estimation of Aruco ")
            return -1   #二维码的marker_length要大，而且marker_length参数要给对,不然没有位姿信息

        _z = data_[0][0][2]
        _ry = data_[0][0][4]
        _perc = data_[0][0][6][0]/960.0 # 归一化[0,1],(892, 292)为二维码中心点坐标
        return (_z, _ry, _perc) # 返回深度z,pitch,画面分辨率(960)的中心点
    else:
        return -1


if __name__=='__main__':
    pass
