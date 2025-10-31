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

# 全局变量用于线程间通信
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
		res = aruco_detector.process_qr_data()
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
		res = aruco_detector.process_qr_data()
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
			res = aruco_detector.process_qr_data()
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
    time_gap = 0.64 #旋转的时间

    #pre read some data 
    rot_once(1,1,0)
    print("start_slow_rot")
    for i in range(slow_rot_times):
        res = aruco_detector.process_qr_data()

        if res != -1:
            _perc = res[2]
            if _perc < 0.4: #让摄像头中心对齐画面分辨率中心
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
        res = aruco_detector.process_qr_data()
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
    time_gap = 0.50 #平移的时间

    for i in range(times):
        res = aruco_detector.process_qr_data()

        if res != -1:
            _perc = res[2]
            if _perc < 0.4: #让摄像头中心对齐画面分辨率中心
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
    
    res = aruco_detector.process_qr_data()

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
    
# 二维码位置信息读取线程
def qr_data_thread():
    global qr_data, stop_threads, task_completed
    print("QR detection thread started")
    
    # 二维码丢失检测计数器
    qr_not_found_count = 0
    # 最大未检测到次数（超过后退出）
    max_not_found_count = 20  # 2秒（0.1秒/次 * 20次）
    
    try:
        while not stop_threads:
            try:
                # 获取aruco二维码信息
                res = aruco_detector.process_qr_data() 
                
                with lock:
                    if res != -1:
                        # 成功检测到二维码，重置丢失计数
                        qr_data['distance'] = res[0]  # 摄像头到二维码的距离
                        qr_data['angle'] = res[1]     # 摄像头到二维码的角度
                        qr_data['percent'] = res[2]   # 二维码在画面中的位置百分比
                        qr_data['found'] = True
                        qr_not_found_count = 0
                    else:
                        # 未检测到二维码，增加丢失计数
                        qr_data['found'] = False
                        qr_not_found_count += 1
                
                # 连续未检测到二维码次数过多，退出线程
                if qr_not_found_count >= max_not_found_count:
                    print(f"QR code not detected for {qr_not_found_count} consecutive times, setting stop flag.")
                    stop_threads = True
                    break
                
                # 检查任务是否已完成
                if task_completed:
                    stop_threads = True
                    break
                
            except Exception as e:
                print(f"Error in QR data processing: {str(e)}")
                # 发生错误时重置状态
                with lock:
                    qr_data['found'] = False
            
            # 检查是否已设置停止标志
            if stop_threads:
                print("Stop flag detected in QR thread, exiting loop.")
                break
                
            time.sleep(0.1)  # 短暂休眠，降低CPU使用率
    except Exception as e:
        print(f"Critical error in QR thread: {str(e)}")
    finally:
        # 确保设置最终状态
        with lock:
            qr_data['found'] = False
        # 关闭摄像头
        print("Closing camera before exiting QR detection thread")
        aruco_detector.close_camera()
        print("QR detection thread exited")

# 小车接近二维码控制线程
def approach_thread():
    global stop_threads, task_completed
    print(f"Approach thread started, initial stop_threads: {stop_threads}")
    
    # 连续未检测到二维码的计数
    qr_not_found_count = 0
    # 最大未检测到次数（超过后退出，防止无限循环）
    max_not_found_count = 15  # 3秒（0.2秒/次 * 15次）
    
    # 需要连续确认的次数，避免单次错误检测导致提前或延迟结束
    target_confirm_count = 3
    reached_target_count = 0
    
    try:
        while not stop_threads:
            with lock:
                current_data = qr_data.copy()
            
            if current_data['found']:
                # 重置未检测到计数
                qr_not_found_count = 0
                
                l = current_data['distance']
                ag = current_data['angle']
                print(f"Distance: {l:.1f} cm, Angle: {ag:.1f}")
                
                # 当距离大于3cm时，控制小车接近
                if l > 5:
                    # 如果之前已经到达过目标距离，现在又远离了，不应该继续接近
                    if reached_target_count > 0:
                        print("Target moved away after being reached. Task completed.")
                        task_completed = True
                        stop_threads = True
                        break
                        
                    # 保持现有的对齐逻辑
                    if stage_slow_rot(6):  # 如果对齐二维码
                        with lock:
                            current_data = qr_data.copy()
                        if current_data['found']:
                            # 根据距离调整前进时间
                            if l > 30:
                                front_once(2, 0.01)  # 远距离前进时间长
                            elif 20 < l <= 30:
                                front_once(1.2, 0.01)  # 中距离前进时间适中
                            elif 5 < l <= 20:
                                front_once(0.5, 0.01)  # 近距离前进时间短
                    else:
                        stages_rot(1, 2, 4)  # 没对齐二维码旋转对齐
                else:
                    # 距离小于等于3cm，增加计数
                    reached_target_count += 1
                    print(f"Reached target distance ({l:.1f} cm), count: {reached_target_count}/{target_confirm_count}")
                    
                    # 连续多次检测都达到目标距离，确认任务完成
                    if reached_target_count >= target_confirm_count:
                        print("Target distance confirmed, task completed.")
                        task_completed = True
                        stop_threads = True
                        break
            else:
                # 未检测到二维码，增加计数
                qr_not_found_count += 1
                print(f"QR code not found, count: {qr_not_found_count}/{max_not_found_count}")
                
                # 连续未检测到二维码次数过多，退出线程
                if qr_not_found_count >= max_not_found_count:
                    print(f"QR code not detected for {qr_not_found_count} consecutive times, exiting thread.")
                    stop_threads = True
                    break
            
            # 检查是否已设置停止标志
            if stop_threads:
                print("Stop flag detected, exiting loop.")
                break
                
            time.sleep(0.2)  # 控制循环频率
    except Exception as e:
        print(f"Error in approach thread: {str(e)}")
        stop_threads = True
    finally:
        # 确保小车停止
        pub_vel(0, 0, 0)
        print("Approach thread exited")

def main_process(first_dir = 1):
    global stop_threads, task_completed
    
    # 设置初始状态
    stop_threads = False
    task_completed = False
    camera_initialized = False
    
    try:
        # 初始化摄像头
        if not aruco_detector.init_camera():
            print("Failed to initialize camera")
            return 0
        camera_initialized = True
        
        #setup camera
        rot_once(1, 3, 0, 0)

        print("Step 1")
        Horizontal_movement(6)  # 水平平移，让画面中心对齐ArUco码
        
        print("Step 2")
        # step 2: rotation and point
        if stages_rot(first_dir, 2, 5) == 0:  # 向右旋转2次，向左旋转5次，画面中心对齐ArUco码
            print("Initial found failed")        
            return 0

        print("Step 3: Starting dual-thread approach")
        
        # 创建并启动线程
        qr_thread = threading.Thread(target=qr_data_thread)
        approach_thread_obj = threading.Thread(target=approach_thread)
        
        qr_thread.start()
        approach_thread_obj.start()
        
        # 等待接近线程完成
        approach_thread_obj.join()
        
        # 停止二维码读取线程
        stop_threads = True
        qr_thread.join()
        
        print("Task completed successfully!")
    except Exception as e:
        print(f"Error in main process: {str(e)}")
        stop_threads = True
    finally:
        pub_vel(0, 0, 0)  # 确保小车停止运动
        # 只在main_process的finally中关闭摄像头，避免重复关闭
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
        pub_vel(0, 0, 0)  # 确保小车停止运动
    except Exception as e:
        print(f"Unexpected error in main: {str(e)}")
        stop_threads = True
        aruco_detector.close_camera()
        pub_vel(0, 0, 0)
    finally:
        print("Program exiting, ensuring camera is closed...")
        aruco_detector.close_camera()
        pub_vel(0, 0, 0)
