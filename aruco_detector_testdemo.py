#coding=UTF-8
import numpy as np
import math
import cv2
import cv2.aruco as aruco

#######################debug
import threading
import time
#######################debug
 
class CSI_ArucoDetector:
    def __init__(self, sensor_id=0, capture_width=3264, capture_height=2464, display_width=960, display_height=540, 
                 framerate=21, flip_method=0, marker_length=0.04):
        # Initialize the video capture with GStreamer pipeline
        self.gstreamer_pipeline = self._gstreamer_pipeline(sensor_id, capture_width, capture_height, display_width, display_height, framerate, flip_method)
        self.cam = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
    
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.marker_length = marker_length   

        # 摄像头的内部参数矩阵
        self.camera_matrix = np.array([  [785.855437,  0.000000,   451.670922], 
                                    [0.000000,    584.820336, 259.056856],
                                    [0.000000,    0.000000,   1.000000  ]])

        # 畸变系数矩阵
        self.dist_coeffs = np.array(([[0.095135, -0.109279, -0.002513,  -0.002418, 0.000000]]))

        # Rotation matrix for flipping the marker's attitude
        self.R_flip = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float32)

        self.pose_data = [None, None, None, None, None, None]
        self.pose_data_dict = {}

    def _gstreamer_pipeline(self, sensor_id, capture_width, capture_height, display_width, display_height, framerate, flip_method):
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
            f"nvvidconv flip-method={flip_method} ! "
            f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=(string)BGR ! appsink"
        )

    def _is_rotation_matrix(self, R):
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
    
    def _rotation_matrix_to_euler_angles(self, R):
            """
            Calculates rotation matrix to euler angles

            :param R:     rotation matrix
            :return:      [np.array] roll, pitch, yaw
            """
            assert (self._is_rotation_matrix(R))

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

    def displayFrame(self, frame_input):
        cv2.imshow("show", frame_input)
        cv2.waitKey(1)

    def getArucoCode(self, display_mode = True ):
        while True:
            frame_makers = None
            ret, frame = self.cam.read() #获取相机的数据流
            frame = cv2.flip(frame,-1)   #垂直镜像翻转
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
                        aruco_res = self._detect(corners[i:i+1], ids[i][0], frame) #输入四个角点坐标，计算并返回x,y,z (units is cm), roll, pitch, yaw (units is degree),Aruco id
                        if aruco_res != None:
                            res.append(aruco_res)
                        i+=1
                        if display_mode :
                            pass
                print("res:",res)
                # return (res, ids)

            else:
                # no data detected
                frame_makers = frame.copy()
                if display_mode :
                    pass
                # return None

            cv2.imshow("show", frame)
            cv2.waitKey(1)

            key = cv2.waitKey(1 )

            if key == 27:        
                print('esc break...')
                self.cap.release()
                cv2.destroyAllWindows()
                break

    def _detect(self, corners, ids, imgWithAruco):
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
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix,self.dist_coeffs)
                for i in range(rvec.shape[0]):
                    imgWithAruco = cv2.drawFrameAxes(imgWithAruco, self.camera_matrix,self.dist_coeffs, rvec, tvec,self.marker_length)

                    frame_makers =   aruco.drawDetectedMarkers(imgWithAruco.copy(),corners)

                # --- The midpoint displays the ID number
                cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                            int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                cv2.putText(imgWithAruco, "id=" + str(ids), cornerMid,
                            font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                rvec = rvec[0][0]
                tvec = tvec[0][0]
                # --- Print the tag position in camera frame
                str_position = "MARKER Position x=%.4f (cm)  y=%.4f (cm)  z=%.4f (cm)" % (tvec[0] * 100, tvec[1] * 100, tvec[2] * 100)
                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = self._rotation_matrix_to_euler_angles(self.R_flip * R_tc)
                # -- Print the marker's attitude respect to camera frame
                str_attitude = "MARKER Attitude degrees r=%.4f  p=%.4f  y=%.4f" % (
                    math.degrees(roll_marker), math.degrees(pitch_marker),
                    math.degrees(yaw_marker))

                self.pose_data[0] = tvec[0] * 100
                self.pose_data[1] = tvec[1] * 100
                self.pose_data[2] = tvec[2] * 100
                self.pose_data[3] = math.degrees(roll_marker)
                self.pose_data[4] = math.degrees(pitch_marker)
                self.pose_data[5] = math.degrees(yaw_marker)

                self.pose_data_dict[ids] = self.pose_data

                roll_deg = math.degrees(roll_marker)
                pitch_deg = math.degrees(pitch_marker)
                yaw_deg = math.degrees(yaw_marker)

                if abs(yaw_deg)%90.0 < 30:
                    return [tvec[0] * 100, tvec[1] * 100, tvec[2] * 100 , roll_deg, pitch_deg ,yaw_deg, cornerMid]
                else:
                    return None

        else:
            self.pose_data[0] = None
            self.pose_data[1] = None
            self.pose_data[2] = None
            self.pose_data[3] = None

            self.pose_data_dict[0] = self.pose_data
            return None

    def process_qr_data(self):
        while True:
            data = self.getArucoCode(True)
            # print("data_",data_)
            # 例子：data_ ([[13.190542591653859, 0.6577785493956305, 30.57489950772101, 56.677235927555834, -10.703176777425517, -17.524720968623765, (892, 292)]], array([[2]], dtype=int32))
            
            if data is not None:
                if data[0] == []:
                    print("can't not dectet pose estimation of Aruco ") #二维码的marker_length要大，而且marker_length参数要给对,不然没有位姿信息
                    return -1   

                _z = data[0][0][2]
                _ry = data[0][0][4]
                _perc = data[0][0][6][0]/960.0 # 归一化[0,1],(892, 292)为二维码中心点坐标
                return (_z, _ry, _perc) # 返回深度z,pitch,画面分辨率(960)的中心点
            else:
                return -1   

# def get_qr_distance(depth_lock, qr_depth):
#     while True:
#         # 这部分代码获取二维码信息，更新深度
#         res = aruco_detector.process_qr_data()  # 获取二维码数据
#         if res != -1:
#             l = res[0]  # 距离
#             ag = res[1]  # 角度

#             # 使用锁来更新共享的二维码深度
#             with depth_lock:
#                 qr_depth["distance"] = l  # 更新二维码的深度

#             print(f"二维码深度更新为：{l}, 角度为：{ag}")
#         else:
#             print("Can't detect aruco.")
        
#         # 控制频率，可以设置为每0.1秒获取一次数据
#         time.sleep(0.1)

# def control_robot(depth_lock, qr_depth):
#     while True:
#         with depth_lock:  # 使用锁来保证对共享数据 qr_depth 的安全访问
#             l = qr_depth["distance"]  # 获取共享的二维码深度
#             print(f"获取到的深度：{l}")

#         if l < 5:
#             # 如果距离小于5，停止运动
#             print("二维码太近，停止小车")
#             break
        
#         elif 5 <= l < 10:
#             # 如果距离在 5 到 10 之间，前进
#             print("前进 0.21 秒")
        
#         elif 10 <= l < 30:
#             # 如果距离在 10 到 30 之间，前进
#             print("前进 1.9 秒")
        
#         elif l >= 30:
#             # 如果距离大于 30，前进
#             print("前进 2 秒")

#         # 控制循环频率，可以设置为每0.1秒获取一次数据
#         time.sleep(0.1)


if __name__=='__main__':
    aruco_detector = CSI_ArucoDetector()

    print("11111111111111111111111")
    aruco_detector.getArucoCode()
    print("22222222222222222222222")
    # # 定义共享数据和锁
    # qr_depth = {"distance": 0}  # 用字典存储二维码的深度
    # depth_lock = threading.Lock()  # 创建一个锁，用于同步对共享数据的访问

    # # 开启线程来实时获取二维码的深度信息
    # qr_thread = threading.Thread(target=get_qr_distance, args=(depth_lock, qr_depth))
    # qr_thread.daemon = True  # 设置为守护线程，主线程退出时会自动退出
    # qr_thread.start()

    # # 开启线程来实时控制小车运动
    # control_thread = threading.Thread(target=control_robot, args=(depth_lock, qr_depth))
    # control_thread.daemon = True  # 设置为守护线程，主线程退出时会自动退出
    # control_thread.start()

    # # 主线程可以做其他工作，或者等待两个子线程结束
    # qr_thread.join()  # 阻塞主线程，直到获取二维码的线程结束
    # control_thread.join()  # 阻塞主线程，直到控制小车的线程结束