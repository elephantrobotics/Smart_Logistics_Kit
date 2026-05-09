# coding=utf8
import rospy
import time
import threading
import numpy as np
import aruco_detector

from pickle import TRUE
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

DETECT = False

aruco_detector_res = None
ids = None
_id_get = 0
qr_data = {'distance': 0, 'angle': 0, 'percent': 0, 'found': False}
stop_threads = False
task_completed = False
lock = threading.Lock()

rospy.init_node('qcode_detect', anonymous=True)
rate = rospy.Rate(30)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

def rot_once(_dir = 1, time_gap_input = 0.5, sp = 0.9, notIgnoreQR = True):
	pub_vel(0, 0, sp*_dir)

	time_ini = time.time()
	while True:
		_time_gap = time.time() - time_ini
		res = aruco_detector.process_qr_data_2()
		if res != -1 and notIgnoreQR:
			#print ("res is " + str(res))
			stop()
			return 1
          	
		if _time_gap > time_gap_input:
			stop()
			return 0

def horizontal_rot_once(_dir = 1, time_gap_input = 0.5, sp = 0.9, notIgnoreQR = True):
	pub_vel(0, sp*_dir, 0)

	time_ini = time.time()
	while True:
		_time_gap = time.time() - time_ini
		res = aruco_detector.process_qr_data_2()
		if res != -1 and notIgnoreQR:
			#print ("res is " + str(res))
			stop()
			return 1
          	
		if _time_gap > time_gap_input:
			stop()
			return 0

def stage_quick_rot(fir_dir = 1, first_rot_times = 3, second_rot_times = 6):
	time_wait = 1
    
	print ("Start stage quick rot")

	def rot_dir_times(_dir,_times):
		for i in range(_times):
			# check qr first 
			res = aruco_detector.process_qr_data_2()
			if res != -1:
				return 1

			# check rotation once then
			if rot_once(_dir):
				return 1

			rot_once(1,time_wait,0,0)
			
		print ("Nothing find in this round")		
		return 0

	if rot_dir_times(fir_dir,first_rot_times) == 1:
		print("counter clock found")
		return 1
	if rot_dir_times(-fir_dir,second_rot_times) == 1:
		print("clock found")
		return 1
	print ("nothing found")
	return 0
			
def stage_slow_rot(slow_rot_times = 6):
    _dir = 1
    sp = 0.5
    time_gap = 0.64 

    #pre read some data 
    rot_once(1,1,0)
    print("start_slow_rot")
    for i in range(slow_rot_times):
        res = aruco_detector.process_qr_data_2()

        if res != -1:
            _perc = res[2]
            if _perc < 0.4: 
                _dir = 1
            elif _perc > 0.6:
                _dir = -1
            else:
                print ("slow move sucess ")
                stop()
                return 1

            rot_once(_dir, time_gap, sp ,notIgnoreQR=False)
            
        else:
            if rot_once(1,1,0) != 1:
                print ("miss the target")
                return -1           
            
        rot_once(1, 1 ,0,0)
        
    print ("slow focus fail")
    return 0   

def front_once(time_gap = 0.5, sp = 0.32):  #20 cm for 0.32sp with 0.5sec
    pub_vel(sp,0, 0)

    time_ini = time.time()
    while True:
        _time_gap = time.time() - time_ini
        res = aruco_detector.process_qr_data_2()
        if _time_gap > time_gap:
            stop()
            return 0    
    stop()
    return 0 # nothing found

def stages_rot(_dir = 1, _first_dir_times = 3, _second_dir_times = 6):
    if stage_quick_rot(_dir, _first_dir_times, _second_dir_times):
        if stage_slow_rot(9):
            return 1
    return 0

def Horizontal_movement(times = 6):
    sp = 0.2
    time_gap = 0.50

    for i in range(times):
        res = aruco_detector.process_qr_data_2()

        if res != -1:
            _perc = res[2]
            if _perc < 0.4: 
                _dir = 1
            elif _perc > 0.6:
                _dir = -1
            else:
                print ("horizontal move sucess ")
                stop()
                return 1

            horizontal_rot_once(_dir, time_gap, sp ,notIgnoreQR=False)
            
        else:
            if rot_once(1,1,0) != 1:
                print ("miss the target")
                return -1           
            
        rot_once(1, 1 ,0,0)
        
def move_to_center():

    _dir = 1
    center_range = 25
    l_time_ratio = 1.4 # important
    
    #pre read some data 
    rot_once(1,1,0,1)
    
    res = aruco_detector.process_qr_data_2()

    if res != -1:
        l, angle = res[0] , res[1]
        print ("Step 2 :  found angle is " + str(angle))
        if angle > center_range :
            _dir = -1
        elif angle < -center_range:
            _dir = 1
        else:
            return 1  # 0 means done need to move
        
        #rotate and go forward
        rot_once(_dir, time_gap_input = 1, sp = 1, notIgnoreQR = False)
        front_once(time_gap= l/100 * l_time_ratio , sp = 0.5)
        
        #rotate back
        rot_once(-_dir, time_gap_input = 0.8, sp = 1, notIgnoreQR = False)
        if stages_rot(-_dir,4,6) == 0:
            return 0
        
        return 1
    else:
        print ("miss target")
        return 0
    
def qr_data_thread():
    global qr_data, stop_threads, task_completed
    print("QR detection thread started")
    
    qr_not_found_count = 0

    max_not_found_count = 20  
    
    try:
        while not stop_threads:
            try:
                res = aruco_detector.process_qr_data_2() 
                
                with lock:
                    if res != -1:
                        qr_data['distance'] = res[0] 
                        qr_data['angle'] = res[1]   
                        qr_data['percent'] = res[2]  
                        qr_data['found'] = True
                        qr_not_found_count = 0
                    else:
                        qr_data['found'] = False
                        qr_not_found_count += 1
                
                if qr_not_found_count >= max_not_found_count:
                    print(f"QR code not detected for {qr_not_found_count} consecutive times, setting stop flag.")
                    stop_threads = True
                    break

                if task_completed:
                    stop_threads = True
                    break
                
            except Exception as e:
                print(f"Error in QR data processing: {str(e)}")
                with lock:
                    qr_data['found'] = False

            if stop_threads:
                print("Stop flag detected in QR thread, exiting loop.")
                break
                
            time.sleep(0.1)  
    except Exception as e:
        print(f"Critical error in QR thread: {str(e)}")
    finally:
        with lock:
            qr_data['found'] = False
        print("Closing camera before exiting QR detection thread")
        aruco_detector.close_camera()
        print("QR detection thread exited")

def approach_thread():
    global stop_threads, task_completed
    print(f"Approach thread started, initial stop_threads: {stop_threads}")
    
    qr_not_found_count = 0
    max_not_found_count = 15  

    target_confirm_count = 3
    reached_target_count = 0
    
    previous_distance = None  
    distance_stability_count = 0  
    distance_stability_threshold = 10  
    distance_diff_threshold = 0.9  
    
    try:
        while not stop_threads:
            with lock:
                current_data = qr_data.copy()
            
            if current_data['found']:
                qr_not_found_count = 0
                
                l = current_data['distance']
                ag = current_data['angle']
                print(f"Distance: {l:.1f} cm, Angle: {ag:.1f}")

                if previous_distance is not None and previous_distance > 0:
                    distance_diff = abs(l - previous_distance)
                    if 0 <= distance_diff <= distance_diff_threshold:
                        distance_stability_count += 1
                        print(f"Distance difference: {distance_diff:.1f} cm, stability count: {distance_stability_count}/{distance_stability_threshold}")
                    else:
                        distance_stability_count = 0
                        print(f"Distance difference: {distance_diff:.1f} cm exceeds threshold, resetting stability count")

                previous_distance = l
                
                if l > 5:
                    if reached_target_count > 0:
                        print("Target moved away after being reached. Task completed.")
                        task_completed = True
                        stop_threads = True
                        break
                        
                    if stage_slow_rot(6): 
                        with lock:
                            current_data = qr_data.copy()
                        if current_data['found']:
                            if l > 30:
                                front_once(2, 0.01) 
                            elif 20 < l <= 30:
                                front_once(1.2, 0.01)  
                            elif 5 < l <= 20:
                                front_once(0.5, 0.01)  
                    else:
                        stages_rot(1, 2, 4) 
                else:
                    reached_target_count += 1
                    print(f"Reached target distance ({l:.1f} cm), count: {reached_target_count}/{target_confirm_count}")
                    
                    if reached_target_count >= target_confirm_count:
                        print("Target distance confirmed, task completed.")
                        task_completed = True
                        stop_threads = True
                        break
            else:
                qr_not_found_count += 1
                print(f"QR code not found, count: {qr_not_found_count}/{max_not_found_count}")
                
                if qr_not_found_count >= max_not_found_count:
                    print(f"QR code not detected for {qr_not_found_count} consecutive times, exiting thread.")
                    stop_threads = True
                    break
            
            if stop_threads:
                print("Stop flag detected, exiting loop.")
                break
                
            time.sleep(0.2)
    except Exception as e:
        print(f"Error in approach thread: {str(e)}")
        stop_threads = True
    finally:
        pub_vel(0, 0, 0)
        print("Approach thread exited")

def main_process(first_dir = 1):
    global stop_threads, task_completed
    
    stop_threads = False
    task_completed = False
    camera_initialized = False
    
    try:
        if not aruco_detector.init_camera():
            print("Failed to initialize camera")
            return 0
        camera_initialized = True
        
        #setup camera
        rot_once(1, 3, 0, 0)

        print("Step 1")
        Horizontal_movement(6)  
        
        print("Step 2")
        # step 2: rotation and point
        if stages_rot(first_dir, 2, 5) == 0:  
            print("Initial found failed")        
            return 0

        print("Step 3: Starting dual-thread approach")
        
        qr_thread = threading.Thread(target=qr_data_thread)
        approach_thread_obj = threading.Thread(target=approach_thread)
        
        qr_thread.start()
        approach_thread_obj.start()
        
        approach_thread_obj.join()
        
        stop_threads = True
        qr_thread.join()
        
        print("Task completed successfully!")
    except Exception as e:
        print(f"Error in main process: {str(e)}")
        stop_threads = True
    finally:
        pub_vel(0, 0, 0) 
        if camera_initialized:
            print("Closing camera in main process finally block")
            aruco_detector.close_camera()

if __name__=='__main__':
    try:
        print("Starting AGV ArUco tracking with dual-thread approach")
        main_process(first_dir = -1)
        print("AGV ArUco tracking completed")
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, stopping threads...")
        stop_threads = True
        aruco_detector.close_camera()
        pub_vel(0, 0, 0)  
    except Exception as e:
        print(f"Unexpected error in main: {str(e)}")
        stop_threads = True
        aruco_detector.close_camera()
        pub_vel(0, 0, 0)
    finally:
        print("Program exiting, ensuring camera is closed...")
        aruco_detector.close_camera()
        pub_vel(0, 0, 0)
