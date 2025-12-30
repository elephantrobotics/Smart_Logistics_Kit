import cv2
from paddleocr import PaddleOCR
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import time
import logging
logging.getLogger('ppocr').setLevel(logging.WARNING)

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

class OCRVideoCapture:
    def __init__(self, font_path="./SIMFANG.TTF", font_size=40):
        logging.basicConfig(level=logging.ERROR)
        self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')
        self.font = ImageFont.truetype(font_path, font_size)
        self.text_color = (0, 255, 0)
        self.time_out=30
        self.start_time=time.time()

    def process_frame(self, raw_frame):
        result = self.ocr.ocr(raw_frame, cls=True)
        if result!=[None]:
            for line in result:
                for word_info in line:
                    text = word_info[1][0]
                    confidence = word_info[1][1]
                    
                    box = word_info[0]
                    cv2.polylines(raw_frame, [np.array(box).astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
                    pil_image = Image.fromarray(raw_frame)
                    draw = ImageDraw.Draw(pil_image)

                    bbox = draw.textbbox((0, 0), text, font=self.font)  
                    box_x0, box_y0, box_x1, box_y1 = bbox  
                    text_width = box_x1 - box_x0
                    text_height = box_y1 - box_y0

                    center_x = (box[0][0] + box[2][0]) / 2
                    center_y = (box[0][1] + box[2][1]) / 2
                    text_x = center_x - text_width / 2
                    text_y = center_y - text_height / 2

                    draw.text((text_x, text_y), text, font=self.font, fill=self.text_color)
                 
                    char_frame = np.array(pil_image)

                    return [char_frame,text]

    def start_capture(self):
        window_title = "CSI Camera"
        video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        if video_capture.isOpened():
            try:
                window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                while True:
                    # Get video stream
                    ret, frame = video_capture.read()
                    flipped_frame = cv2.flip(frame,-1)
                    if not ret:
                        print("Failed to capture video stream")
                        break

                    result = self.process_frame(flipped_frame)
        
                    if result:
                        cv2.imshow("Char Scanner", result[0])
                        cv2.waitKey(1500)
                        cv2.destroyAllWindows()
                        return result[1]
                    else:
                        cv2.imshow("Char Scanner",flipped_frame)
                    
                        # Press 'q' to quit
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            cv2.destroyAllWindows()
                            break
                    if time.time()-self.start_time>self.time_out:
                        print("Timeout, failed to recognize characters")
                        cv2.destroyAllWindows()
                        return -1
            finally:
                video_capture.release()
                cv2.destroyAllWindows()

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()

#Use Case
if __name__ == "__main__":
    ocr_capture = OCRVideoCapture()  
    for i in range(5):
        print(ocr_capture.start_capture())
