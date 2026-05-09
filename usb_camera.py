import cv2
import time
from pymycobot.mecharm270 import MechArm270
mc = MechArm270('/dev/ttyACM0',115200)
mc.set_fresh_mode(0)
# mc.release_all_servos()

# mc.send_angles([94.13, 10.2, -21.88, 0.96, 90.79, 0.0], 50)
mc.send_angles([-93.36, 20.03, -22.85, -3.07, 89, 1.46],50)
cap = cv2.VideoCapture("/dev/video1") 

if not cap.isOpened():
    print("Unable to open the camera")
    exit()

while True:
    ret, frame = cap.read()

    if ret:
        cv2.imshow('Camera Feed', frame)

    else:
        print("Failed to read frame from camera")
        break
    print(mc.get_angles())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()