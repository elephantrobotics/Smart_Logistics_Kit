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
	#print (_dir, sp , time_gap_input)
	time_ini = time.time()
	while True:
		_time_gap = time.time() - time_ini
		res = aruco_detector.process_qr_data()
		if res != -1 and notIgnoreQR:
			#print ("res is " + str(res))
			stop()
			return 1
			break		
		if _time_gap > time_gap_input:
			stop()
			return 0
			break

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
    time_gap = 0.50 #旋转的时间

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

def control_robot(depth_lock, qr_depth):
    while True:
        with depth_lock:  # 使用锁来保证对共享数据 qr_depth 的安全访问
            l = qr_depth["distance"]  # 获取共享的二维码深度
            print(f"获取到的深度：{l}")

        if l < 5:
            # 如果距离小于5，停止运动
            print("二维码太近，停止小车")
            pub_vel(0, 0, 0)  # 停止小车运动
            break
        elif 5 <= l < 10:
            # 如果距离在 5 到 10 之间，前进
            print("前进 0.21 秒")
            front_once(0.21, sp=0.01)
        elif 10 <= l < 30:
            # 如果距离在 10 到 30 之间，前进
            print("前进 1.9 秒")
            front_once(1.9, sp=0.01)
        elif l >= 30:
            # 如果距离大于 30，前进
            print("前进 2 秒")
            front_once(2, sp=0.01)

        # 控制循环频率，可以设置为每0.1秒获取一次数据
        time.sleep(0.1)

def get_qr_distance(depth_lock, qr_depth):
    while True:
        # 这部分代码获取二维码信息，更新深度
        res = aruco_detector.process_qr_data()  # 获取二维码数据
        if res != -1:
            l = res[0]  # 距离
            ag = res[1]  # 角度

            # 使用锁来更新共享的二维码深度
            with depth_lock:
                qr_depth["distance"] = l  # 更新二维码的深度

            print(f"二维码深度更新为：{l}, 角度为：{ag}")
        else:
            print("Can't detect aruco.")
        
        # 控制频率，可以设置为每0.1秒获取一次数据
        time.sleep(0.1)

def main_process(first_dir = 1):

    #setup camera
    rot_once(1,3,0,0)

    print ("Step 1")
    # step 1: rotation and point
    if stages_rot(first_dir,2,5) == 0: # 向右旋转2次，向左旋转5次，画面中心对齐ArUco码
        print ("initial found failed")        
        return 0

    # print ("Step 2")
    # step 2: move to center
    # if move_to_center() == 0:  #让小车odom跟ArUco码对齐同一直线，目前效果不好
    #    print ("Target not found") 
    #    return 0

    print ("Step 2")   
    # step 2: 
    while True:
        res = aruco_detector.process_qr_data() #获取aruco二维码信息
        if res != -1:
            l = res[0] # 摄像头到二维码的距离
            ag = res[1]# 摄像头到二维码的角度
            print ("l is " + str(l) + " angle is " + str(ag))

            if 30 < l:
                if stage_slow_rot(6):   #如果对齐二维码
                    front_once(2,0.01)#前进时间长
                    continue
                else:
                    stages_rot(1,2,4)   #没对齐二维码旋转对齐   

            elif 10 < l < 30 :      
                if stage_slow_rot(6):   #如果对齐二维码如果此时扫描不到二维码还是会前进
                    front_once(1.9,0.01)  #前进#bug不及时
                    continue
                else:
                    stages_rot(1,2,4)   #没对齐二维码旋转对齐

            elif 5 < l < 10:
                #one time up
                front_once(0.21, sp=0.01)#前进0.21秒
                print ("Finsih doing") 
                continue

            elif l < 5:
                # rot_once(1,1,0,0)
                break
                
        else:#获取不到aruco二维码信息就退出循环
            print("Can't detect aruco.")
            pub_vel(0,0,0)
            break
    
    rot_once(1,1,0,0) #停止运动

###################################################################################################debug
    # # 定义共享数据和锁
    # qr_depth = {"distance": 0}  # 用字典存储二维码的深度
    # depth_lock = threading.Lock()  # 创建一个锁，用于同步对共享数据的访问

    # # 开启线程来实时获取二维码的深度信息
    # qr_thread = threading.Thread(target=get_qr_distance, args=(depth_lock, qr_depth))
    # qr_thread.daemon = True  # 设置为守护线程，主线程退出时会自动退出
    # qr_thread.start()

    # # 开启线程来实时控制小车运动
    # control_thread = threading.Thread(target=control_robot, args=(depth_lock, qr_depth))
    # control_thread.daemon = True  # 设置为守护线程，主线程退出时会自动退出
    # control_thread.start()

    # # 主线程可以做其他工作，或者等待两个子线程结束
    # qr_thread.join()  # 阻塞主线程，直到获取二维码的线程结束
    # control_thread.join()  # 阻塞主线程，直到控制小车的线程结束
###################################################################################################

if __name__=='__main__':
    try:
        print ("The main process would be " + str(main_process(first_dir = -1)) )
    except rospy.exceptions.ROSException as e:
        print("Node has already been initialized, do nothing")
