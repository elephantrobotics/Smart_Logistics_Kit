import cv2
import cv2.aruco as aruco
import threading
import queue
import numpy as np
import math

class CameraProcessor:
    def __init__(self, camera_matrix, dist_matrix, marker_length=0.04):
        self.frame_queue_capture = queue.Queue(maxsize=2)  # Queue for capturing frames
        self.frame_queue_process = queue.Queue(maxsize=2)  # Queue for processing frames
        self.pose_data = [None, None, None, None, None, None]
        self.pose_data_dict = {}
        
        self.camera_matrix = camera_matrix
        self.dist_matrix = dist_matrix
        self.marker_length = marker_length
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0
    
    # GStreamer pipeline functions
    @staticmethod
    def gstreamer_pipeline(sensor_id=0, capture_width=3264, capture_height=2464, display_width=960, display_height=540, framerate=21, flip_method=0):
        return (
            "nvarguscamerasrc sensor-id=%d !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )
    
    def _is_rotation_matrix(self, R):
        """
        Checks if a matrix is a valid rotation matrix.
        """
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def _rotation_matrix_to_euler_angles(self, R):
        """
        Calculates rotation matrix to euler angles
        """
        assert self._is_rotation_matrix(R)
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
    
    def _detect(self, corners, ids, imgWithAruco):
        """
        Show the Axis of aruco and return the x,y,z(unit is cm), roll, pitch, yaw
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
            if ids is not None:  # if aruco marker detected
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_matrix)
                for i in range(rvec.shape[0]):
                    imgWithAruco = cv2.drawFrameAxes(imgWithAruco, self.camera_matrix, self.dist_matrix, rvec, tvec, self.marker_length)
                    
                # --- The midpoint displays the ID number
                cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4), int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))

                cv2.putText(imgWithAruco, "id=" + str(ids), cornerMid,
                            font, 1, (255, 255, 255), 1, cv2.LINE_AA)

                rvec = rvec[0][0]
                tvec = tvec[0][0]
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T
                roll_marker, pitch_marker, yaw_marker = self._rotation_matrix_to_euler_angles(self.R_flip * R_tc)

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

                if abs(yaw_deg) % 90.0 < 30:
                    return [tvec[0] * 100, tvec[1] * 100, tvec[2] * 100, roll_deg, pitch_deg, yaw_deg, cornerMid]
                else:
                    return None
        else:
            return None

    def capture_frames(self, cap):
        while True:
            ret_val, frame = cap.read()
            if ret_val:
                if self.frame_queue_capture.full():
                    self.frame_queue_capture.get()  # Discard the oldest frame in the queue
                self.frame_queue_capture.put(frame)

    def process_frames(self):
        while True:
            if not self.frame_queue_capture.empty():
                frame = self.frame_queue_capture.get()

                frame = cv2.flip(frame, -1)  # Vertical flip
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Grayscale

                aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
                parameters = cv2.aruco.DetectorParameters()

                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    ids_len = len(ids)
                    res = []
                    for i in range(ids_len):
                        aruco_res = self._detect(corners[i:i + 1], ids[i][0], frame)
                        if aruco_res is not None:
                            res.append(aruco_res)

                    if res:
                        _z = res[0][2]
                        _ry = res[0][4]
                        _perc = res[0][6][0] / 960.0  # Normalized to [0,1]
                        print("Z:", _z)
                        print("ry", _ry)
                        print("perc", _perc)
                        
                # Put the processed image into the processing queue
                if not self.frame_queue_process.full():
                    self.frame_queue_process.put(frame)

            # Check if the user has closed the window
            keyCode = cv2.waitKey(1) & 0xFF
            if keyCode == 27 or keyCode == ord('q'):  # ESC key or 'q' key to exit
                break

        cv2.destroyAllWindows()

    def display_processed_frames(self, frame_queue_process):
        window_title = "CSI Camera (Processed)"
        while True:
            if not frame_queue_process.empty():
                frame = frame_queue_process.get()
                cv2.imshow(window_title, frame)

            keyCode = cv2.waitKey(1) & 0xFF
            if keyCode == 27 or keyCode == ord('q'):  # Press the ESC key or the 'q' key to exit
                break

    def show_camera(self):
        video_capture = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        if video_capture.isOpened():
            capture_thread = threading.Thread(target=self.capture_frames, args=(video_capture,))
            process_thread = threading.Thread(target=self.process_frames)
            display_thread = threading.Thread(target=self.display_processed_frames, args=(self.frame_queue_process,))
            
            capture_thread.start()
            process_thread.start()
            display_thread.start()

            capture_thread.join()
            process_thread.join()
            display_thread.join()
            video_capture.release()
        else:
            print("Error: Unable to open camera")

# if __name__ == "__main__":
#     camera_matrix = np.array([[785.855437, 0.000000, 451.670922], 
#                               [0.000000, 584.820336, 259.056856],
#                               [0.000000, 0.000000, 1.000000]], dtype=np.float32)
#     dist_matrix = np.array([0.095135, -0.109279, -0.002513, -0.002433, 0.000000], dtype=np.float32)

#     camera_processor = CameraProcessor(camera_matrix, dist_matrix)
#     camera_processor.show_camera()