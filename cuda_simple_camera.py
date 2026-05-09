import cv2
import cv2.aruco as aruco
import threading
import queue
import numpy as np
import math
import time
# from paddleocr import PaddleOCR

# from PIL import Image, ImageDraw, ImageFont
# import logging


class CameraProcessor:
    def __init__(self, 
                 camera_matrix=np.array([[785.855437, 0.000000, 451.670922], 
                                         [0.000000, 584.820336, 259.056856],
                                         [0.000000, 0.000000, 1.000000]], dtype=np.float32), 
                 dist_matrix=np.array([0.095135, -0.109279, -0.002513, -0.002433, 0.000000], dtype=np.float32),
                 marker_length=0.04):
        
        self.frame_queue_capture = queue.Queue(maxsize=2)  
        self.frame_queue_process = queue.Queue(maxsize=2)  

        # Initialize pose data (translation, rotation, and position)
        self.pose_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0)]
        self.pose_data_dict = {}
        
        # Camera calibration matrix (intrinsics) and distortion coefficients
        self.camera_matrix = camera_matrix
        self.dist_matrix = dist_matrix

        self.marker_length = marker_length
        self.R_flip = np.zeros((3, 3), dtype=np.float32)
        self.R_flip[0, 0] = 1.0
        self.R_flip[1, 1] = -1.0
        self.R_flip[2, 2] = -1.0

        self.loop_mode = True  

        # ocr 
        # logging.getLogger('ppocr').setLevel(logging.WARNING)
        # logging.basicConfig(level=logging.ERROR)
        # self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')
        # self.font = ImageFont.truetype("./SIMFANG.TTF", 40)
        # self.text_color = (0, 255, 0)
        # self.time_out=30
        # self.start_time=time.time()

    @staticmethod
    def gstreamer_pipeline(sensor_id=0, capture_width=3264, capture_height=2464, display_width=960, display_height=540, framerate=21, flip_method=2):
        return (
            "nvarguscamerasrc sensor-id=%d !"
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (sensor_id, capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )
    
    def start_imx219_video_capture(self):
        self.video_capture = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

    def stop_IMX219_video_capture(self):
        self.video_capture.release()

    def get_aruco_realtime_date(self):
        return self.pose_data[0], self.pose_data[4], self.pose_data[6][0]/960.0
    
    def set_loop_mode(self,loop_mode):
        self.loop_mode = loop_mode

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

                yaw_deg = math.degrees(yaw_marker)

                if abs(yaw_deg) % 90.0 < 30:
                    self.pose_data[0] = tvec[0] * 100
                    self.pose_data[1] = tvec[1] * 100
                    self.pose_data[2] = tvec[2] * 100
                    self.pose_data[3] = math.degrees(roll_marker)
                    self.pose_data[4] = math.degrees(pitch_marker)
                    self.pose_data[5] = math.degrees(yaw_marker)
                    self.pose_data[6] = cornerMid

                    self.pose_data_dict[ids] = self.pose_data

    def capture_frames(self, cap):
        while self.running:
            ret_val, frame = cap.read()
            if ret_val:
                if self.frame_queue_capture.full():
                    self.frame_queue_capture.get()  
                self.frame_queue_capture.put(frame)

    def process_frames(self):
        while self.running:
            if not self.frame_queue_capture.empty():
                frame = self.frame_queue_capture.get()

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  

                aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
                parameters = cv2.aruco.DetectorParameters()

                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    ids_len = len(ids)
                    for i in range(ids_len):
                        self._detect(corners[i:i + 1], ids[i][0], frame)
             
                if not self.frame_queue_process.full():
                    self.frame_queue_process.put(frame)

            keyCode = cv2.waitKey(1) & 0xFF
            if keyCode == 27 or keyCode == ord('q'):  
                break

        cv2.destroyAllWindows()

    def display_processed_frames(self, frame_queue_process):
        window_title = "CSI Camera (Processed)"
        while self.running:
            if not frame_queue_process.empty():
                frame = frame_queue_process.get()
                cv2.imshow(window_title, frame)
             
                keyCode = cv2.waitKey(1) & 0xFF
                if keyCode == 27 or keyCode == ord('q'): 
                    break
   
    def process_aruco_realtime(self):
        if self.video_capture.isOpened():
            self.running = True  
            capture_thread = threading.Thread(target=self.capture_frames, args=(self.video_capture,)) 
            process_thread = threading.Thread(target=self.process_frames) 
            display_thread = threading.Thread(target=self.display_processed_frames, args=(self.frame_queue_process,)) 
            try:
                capture_thread.start()
                process_thread.start()
                display_thread.start()

                display_thread.join()
                process_thread.join()
                display_thread.join()

            except KeyboardInterrupt:
                print("\nKeyboardInterrupt detected. Stopping threads...")

            finally:
                self.running = False  
                self.stop_IMX219_video_capture()
                cv2.destroyAllWindows()
                print("Camera released and program exited.")
        else:
            print("Error: Unable to open camera")

    def stop_aruco_realtime(self):
        self.running = False 
        self.stop_IMX219_video_capture()
        cv2.destroyAllWindows()
        print("Camera released and program exited.")

if __name__ == "__main__":
    camera_processor = CameraProcessor()
    camera_processor.start_imx219_video_capture()
    camera_processor.process_aruco_realtime()