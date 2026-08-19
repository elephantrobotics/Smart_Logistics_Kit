# coding=utf8
import rospy
import time
import threading
import numpy as np
import sys
from xmlrpc.server import SimpleXMLRPCServer
import aruco_detector
from pickle import TRUE
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist


DETECT = False
robot_state = "IDLE"
shutdown_in_progress = False
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
        aruco_detector.close_camera()
        time.sleep(1)
        return "unload align done"
    elif mode_str == "align":
        detect_func = aruco_detector.process_qr_data_simple
        auto_align_marker()
        aruco_detector.close_camera()
        time.sleep(1)
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
    server.register_function(get_robot_state, "get_robot_state")
    # 1. Initialize camera ONCE at startup (The 10-second tax is paid here)
    print("Initializing camera... please wait.")
    if not aruco_detector.init_camera():
        print("Camera failed!")
        return

    print("RPC Server is blocking the main thread. Ready for calls...")
    
    server.serve_forever()
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
    mode: 'horizontal' or 'rot'
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

TARGET_CENTER = 0.5 
TARGET_YAW = 0.0     

Kp_linear_x = 0.005 
Kp_linear_y = 1.5    
Kp_angular  = 0.015  

MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = 0.5

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def auto_align_marker(): 
    global robot_state
    robot_state = "ALIGNING"
    ANGLE_THRESHOLD = 10.0  
    X_PERC_THRESHOLD = 0.05
    DIST_THRESHOLD = 15.0   # 2cm
    max_not_found_count = 5  # 2s
    qr_not_found_count = 0
    last_known_z = 999.0
    print("Start automatic alignment of Aruco QR code...")
    
    try:
        while True:

            qr_data = detect_func()
            if qr_data is None or qr_data == -1:
                print("No valid marker detected, waiting...",qr_data)
                pub_vel(0, 0, 0)
                time.sleep(0.1)
                qr_not_found_count +=1
                if qr_not_found_count >= max_not_found_count:
                    if last_known_z > 30.0: # If last seen further than 15cm
                        print("FAILED: Lost marker while too far away. Aborting.")
                        pub_vel(0, 0, 0)
                        robot_state = "ALIGNING_FAILED"
                    else:
                        print("SUCCESS: Lost marker in blind spot (Close enough).")
                        pub_vel(0, 0, 0)
                        robot_state = "ALIGNING_SUCCESS"
                    break
                continue
            qr_not_found_count = 0
            cur_z, cur_ry, cur_perc = qr_data
            last_known_z = cur_z
            if cur_z < 7:
                X_PERC_THRESHOLD = 0.14
            elif cur_z < 8:
                X_PERC_THRESHOLD = 0.13
            elif cur_z < 9:
                X_PERC_THRESHOLD = 0.10
            elif cur_z < 10:
                X_PERC_THRESHOLD = 0.09
            elif cur_z < 15:
                X_PERC_THRESHOLD = 0.08
            elif cur_z < 30:
                X_PERC_THRESHOLD = 0.07 

            error_x = TARGET_CENTER - cur_perc    
            error_yaw = TARGET_YAW - cur_ry      
            
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
                    pub_vel(0, 0, vel_theta)
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
 
                vel_y = 0.11 if error_x > 0 else -0.11 # you don't want the speed too small,it will not provide enough torque
                start_time = time.time()
                while time.time() - start_time < 0.8:
                    pub_vel(0, vel_y, 0)
                    time.sleep(0.05) # Send command at 20Hz (standard for ROS)
                pub_vel(0, 0, 0)
                time.sleep(1.4) # Wait for camera to stabilize
                continue
            if cur_z > 5:
                print(f"-> Action: Moving Forward, adjusting distance: {cur_z:.1f}cm")
                if cur_z > 30:
                    pub_vel(0.01,0, 0)
                    time.sleep(1.8)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                elif 20 < cur_z <= 30:
                    pub_vel(0.01,0, 0)
                    time.sleep(1)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                elif 5 < cur_z <= 20:
                    pub_vel(0.01,0, 0)
                    time.sleep(0.5)
                    pub_vel(0, 0, 0)
                    time.sleep(0.3)
                continue
            
            if (abs(error_x) < X_PERC_THRESHOLD and 
                abs(error_yaw) < ANGLE_THRESHOLD and 
                abs(cur_z) <= DIST_THRESHOLD):
                
                print(">>> [SUCCESS] All errors within dead zone. <<<")
                pub_vel(0, 0, 0)
                break

        aruco_detector.closeDisplayFrame()
    except KeyboardInterrupt:
        print("User forced stop.")
        pub_vel(0, 0, 0)
    finally:
        if robot_state!="ALIGNING_FAILED":
            robot_state = "IDLE"
        aruco_detector.closeDisplayFrame()
        print("align done")
def get_robot_state():
    return robot_state
def handle_exit_signal(signal, frame):
    """Handle SIGTERM and SIGINT signals for clean shutdown"""
    global stop_threads, shutdown_in_progress
    if shutdown_in_progress:
        return
    shutdown_in_progress = True
    
    print(f"\nReceived signal {signal}, initiating clean shutdown...")
    stop_threads = True
    stop()
    try:
        aruco_detector.close_camera()
    except:
        pass
    try:
        aruco_detector.closeDisplayFrame()
    except:
        pass
    print("AGV ArUco service shutdown complete")
    sys.exit(0)

if __name__=='__main__':
    import signal as sig_module
    sig_module.signal(sig_module.SIGINT, handle_exit_signal)
    sig_module.signal(sig_module.SIGTERM, handle_exit_signal)
    
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