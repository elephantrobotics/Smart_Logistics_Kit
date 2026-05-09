import cv2

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
        "nvarguscamerasrc sensor-id=%d ! "
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

# Initialize camera
if not init_camera():
    print("Failed to open camera")
    exit()

try:
    while True:
        ret, frame = cam.read()
        if not ret:
            print("Failed to read frame")
            break

        # Display the image
        cv2.imshow("Jetson Camera", frame)

        # Exit on ESC key
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("Ctrl-C detected, exiting gracefully...")

finally:
    if cam is not None:
        cam.release()
    cv2.destroyAllWindows()
