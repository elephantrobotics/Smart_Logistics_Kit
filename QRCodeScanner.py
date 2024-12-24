import cv2
from pyzbar.pyzbar import decode
import numpy as np
import re
from PIL import Image, ImageDraw, ImageFont
import time

class QRCodeScanner:
    def __init__(self, camera_index=1, font_path="./SIMFANG.TTF", font_size=25):
        self.camera_index = camera_index
        self.font_path = font_path
        self.font_size = font_size
        self.font = ImageFont.truetype(self.font_path, self.font_size)
        self.time_out=60
        self.cap = cv2.VideoCapture("/dev/video1") 
        self.text_color = (0, 255, 0)
        if not self.cap.isOpened():
            raise Exception("无法打开摄像头")

    def scan_qrcode_from_camera(self,raw_frame):

        decoded_objects = decode(raw_frame)
        if decoded_objects:
            
            for obj in decoded_objects:
                qr_data = obj.data.decode("utf-8")
                # print(f"{i} QR Code Data: {qr_data}")
                match = re.search(r'地址：(.+?市)',  qr_data)
                if match:
                    city = match.group(1)
                    if "省" in city:
                        parts =city.split("省")
                        city=parts[-1]
                else:
                    print("未找到城市信息")
                points = obj.polygon
                if len(points) == 4:  # 
                    pts = np.array(points, dtype=np.int32)
                    cv2.polylines(raw_frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2)

                    x, y, w, h = cv2.boundingRect(pts)
                    pil_image = Image.fromarray(raw_frame)
                    draw = ImageDraw.Draw(pil_image)
                    bbox = draw.textbbox((x, y), qr_data, font=self.font)
                    text_width = bbox[2] - bbox[0]
                    text_height = bbox[3] - bbox[1]

                    text_x = x + (w - text_width) // 2
                    text_y = y + (h - text_height) // 2

                    draw.text((text_x, text_y), qr_data, font=self.font, fill= self.text_color)
                    qr_frame = np.array(pil_image)
                   
                    return [qr_frame,city]

    def start_capture(self):
        while True:
            self.start_time=time.time()
            ret, frame = self.cap.read()
            if not ret:
                print("无法读取视频流")
                break
            result=self.scan_qrcode_from_camera(frame)
            
            if result:
                cv2.imshow("QR Code Scanner", result[0])
                cv2.waitKey(1500)
                cv2.destroyAllWindows()
                return result[1]
            else:
                cv2.imshow("QR Code Scanner",frame)
            
                # 按 'q' 键退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            print(time.time()-self.start_time)
            if time.time()-self.start_time>self.time_out:
                print("60s识别超时")
                cv2.destroyAllWindows()
                return -1
    
    def release_resources(self):
        # 释放摄像头和窗口
        self.cap.release()
        cv2.destroyAllWindows()

# 使用示例
# if __name__ == "__main__":
#     scanner = QRCodeScanner()
#     for i in range(5):
#         print(scanner.start_capture())
   