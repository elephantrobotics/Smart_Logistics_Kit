import cv2

cap = cv2.VideoCapture("/dev/video1") 

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    # 从摄像头读取一帧
    ret, frame = cap.read()
    
    # 如果读取成功，显示帧
    if ret:
        cv2.imshow('Camera Feed', frame)
    else:
        print("无法获取帧")
        break
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()