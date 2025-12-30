import cv2

cap = cv2.VideoCapture("/dev/video1") 

# Check if the camera has been successfully turned on
if not cap.isOpened():
    print("Failed to open the camera")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    
    # If the read is successful, display the frame
    if ret:
        cv2.imshow('Camera Feed', frame)
    else:
        print("Failed to get frame")
        break
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()