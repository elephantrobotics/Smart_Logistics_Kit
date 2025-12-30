# coding=utf8
import rospy
import time
import threading
import numpy as np
from xmlrpc.server import SimpleXMLRPCServer
import aruco_detector
from pickle import TRUE
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist


DETECT = False

def handle_request(mode_str):
    global detect_func
    print(f"Received command: {mode_str}")
    if mode_str == "check_box_qrcodes":
        detect_func = aruco_detector.process_qr_data_simple

        res = aruco_detector.check_box_qrcodes()
        if isinstance(res, dict) and "is_upper" in res:
            res["is_upper"] = bool(res["is_upper"])
        return res
    elif mode_str == "unload":
        detect_func = aruco_detector.process_qr_data_2
        auto_align_marker()
        return "unload align done"
    elif mode_str == "align":
        detect_func = aruco_detector.process_qr_data_simple
        auto_align_marker()
        return "align done"
    return "Error: Unknown Mode"
aruco_detector_res = None
ids = None
_id_get = 0
qr_data = {'distance': 0, 'angle': 0, 'percent': 0, 'found': False}
stop_threads = False
task_completed = False
lock = threading.Lock()


pub=None
def main_process():
    global pub
    rospy.init_node('qcode_detect', anonymous=True)
    rate = rospy.Rate(30)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    server = SimpleXMLRPCServer(("localhost", 6666), allow_none=True)
    server.register_introspection_functions()
    server.register_function(handle_request, "aruco_rpc")
    
    # 1. Initialize camera ONCE at startup (The 10-second tax is paid here)
    print("Initializing camera... please wait.")
    if not aruco_detector.init_camera():
        print("Camera failed!")
        return

    print("RPC Server is blocking the main thread. Ready for calls...")
    
    server.serve_forever()
# Global variables are used for inter-thread communication.
def pub_vel(x, y , theta):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = theta
    pub.publish(twist)

def stop():
    pub_vel(0,0,0)


def move_check_once(mode, _dir=1, time_gap=0.5, sp=0.9, notIgnoreQR=True):
    """
    mode: 'horizontal' (Translation) or 'rotation'
    """
    if mode == 'horizontal':
        pub_vel(0, sp * _dir, 0)
    elif mode == 'rot':
        pub_vel(0, 0, sp * _dir)
    else:
        print(f"Error: Unknown mode '{mode}'")
        return 0


    time_ini = time.time()
    while True:
        _time_gap = time.time() - time_ini
        res = detect_func()
        if res != -1 and notIgnoreQR:
            # print("res is " + str(res))
            stop()
            return 1
        if _time_gap > time_gap:
            stop()
            return 0


def front_once(time_gap = 0.5, sp = 0.32):  #20 cm for 0.32sp with 0.5sec
    pub_vel(sp,0, 0)

    time_ini = time.time()
    while True:
        _time_gap = time.time() - time_ini
        res = detect_func()
        if _time_gap > time_gap:
            stop()
            return 0    




TARGET_CENTER = 0.5  # Target center: 50% of image width
TARGET_YAW = 0.0     # Target angle: 0 degrees (parallel)

# P controller parameters (need to be adjusted based on actual vehicle speed and response speed)
Kp_linear_x = 0.005   # Front and rear speed coefficients
Kp_linear_y = 1.5    # Lateral velocity coefficient (used for center alignment)
Kp_angular  = 0.015   # Rotational speed coefficient (used to adjust parallelism)

# Speed ​​limit (to prevent excessive speed)
MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = 0.5

def clamp(value, min_val, max_val):
    """Limiting the range of values"""
    return max(min_val, min(value, max_val))

def auto_align_marker(): 
    # Define thresholds
    ANGLE_THRESHOLD = 10.0  
    X_PERC_THRESHOLD = 0.05
    DIST_THRESHOLD = 5.0   # 2cm
    max_not_found_count = 20  # 2 seconds (0.1 seconds/time * 20 times)
    qr_not_found_count = 0
    print("Start automatically aligning Aruco QR codes...")
    
    try:
        while True:

            qr_data = detect_func()
            if qr_data is None or qr_data == -1:
                print("No valid marker detected, stop and wait....",qr_data)
                pub_vel(0, 0, 0)
                time.sleep(0.1)
                qr_not_found_count +=1
                if qr_not_found_count >= max_not_found_count:
                    print(f"QR code not detected for {qr_not_found_count} consecutive times, setting stop flag.")
                    break
                continue
            
            cur_z, cur_ry, cur_perc = qr_data
            if cur_z < 10:
                 X_PERC_THRESHOLD = 0.15
            elif cur_z < 15:
                 X_PERC_THRESHOLD = 0.12
            elif cur_z < 30:
                 X_PERC_THRESHOLD = 0.08 

            error_x = TARGET_CENTER - cur_perc     # >0 indicates the target is on the left, and the car needs to move to the left.
            error_yaw = TARGET_YAW - cur_ry        # Angular error
            
            print(f"Dist:{cur_z:.1f}cm | Yaw:{cur_ry:.1f}° | Pos:{cur_perc:.2f}")

            # vel_theta = error_yaw * Kp_angular 
            if abs(error_yaw) >  ANGLE_THRESHOLD: 
                vel_x = 0
                vel_y = 0
                if error_yaw > 0:
                    vel_theta = -0.1 
                else:
                    vel_theta = 0.1
                print("Prioritizing Rotation...")
                start_time = time.time()
                while time.time() - start_time < 0.8:
                    pub_vel(0, vel_y, vel_theta)
                    time.sleep(0.05)
                pub_vel(0, 0, 0)
                time.sleep(1.4)
            else:

                # vel_x = error_dist * Kp_linear_x
                # vel_y = error_x * Kp_linear_y
                # vel_theta = error_yaw * Kp_angular # Small correction only
                pub_vel(0, 0, 0)

            if abs(error_x) > X_PERC_THRESHOLD:
                print("-> Action: Moving Laterally (X-Axis)")
 
                vel_y = 0.1 if error_x > 0 else -0.1 # you don't want the speed too small,it will not provide enough torque
                start_time = time.time()
                while time.time() - start_time < 0.8:
                    pub_vel(0, vel_y, 0)
                    time.sleep(0.05) # Send command at 20Hz (standard for ROS)
                pub_vel(0, 0, 0)
                time.sleep(1.4) # Wait for camera to stabilize
                continue
            if cur_z > 5:
                print(f"Moving forward and adjusting distance: {cur_z:.1f}cm")
                if cur_z > 30:
                    pub_vel(0.01,0, 0)
                    time.sleep(2)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                elif 20 < cur_z <= 30:
                    pub_vel(0.01,0, 0)
                    time.sleep(1.2)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                elif 5 < cur_z <= 20:
                    pub_vel(0.01,0, 0)
                    time.sleep(0.5)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                continue
            
            # --- Final judgment: All errors are within the dead zone. ---
            if (abs(error_x) < X_PERC_THRESHOLD and 
                abs(error_yaw) < ANGLE_THRESHOLD and 
                abs(cur_z) <= DIST_THRESHOLD):
                
                print(">>> [Success] Angle, center, and distance are all aligned! <<<")
                pub_vel(0, 0, 0)
                break


    except KeyboardInterrupt:
        print("User forced stop")
        pub_vel(0, 0, 0)
    finally:
        print("align done")
if __name__=='__main__':
    try:
        print("Starting AGV ArUco tracking with dual-thread approach")
        main_process()
        print("AGV ArUco tracking completed")
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, stopping threads...")
        stop_threads = True
        aruco_detector.close_camera()
        stop()
    except Exception as e:
        print(f"Unexpected error in main: {str(e)}")
        stop_threads = True
        aruco_detector.close_camera()
        stop()
    finally:
        print("Program exiting, ensuring camera is closed...")
        aruco_detector.close_camera()
        stop()
