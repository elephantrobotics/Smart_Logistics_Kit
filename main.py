#!/usr/bin/env python                                                                                                                      
#coding=UTF-8
import rospy
import time
import actionlib
import signal
import sys
import os
import numpy as np
import Jetson.GPIO as GPIO

from pymycobot.mecharm270 import MechArm270
from pymycobot.utils import get_port_list

from OCRVideoCapture import OCRVideoCapture
from QRCodeScanner import QRCodeScanner
from Transformation import homo_transform_matrix

from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class MapNavigation:
    def __init__(self):
        self.goalReached = False
        rospy.init_node('map_navigation', anonymous=False)
        
        # ros publisher
        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pub_setpose = rospy.Publisher('/initialpose',PoseWithCovarianceStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(19, GPIO.OUT)
        GPIO.setup(26, GPIO.OUT)
        self.pump_off()

    # init robot  pose AMCL
    def set_pose(self, xGoal, yGoal, orientation_z, orientation_w,covariance):
        pose = PoseWithCovarianceStamped()
        pose.header.seq = 0
        pose.header.stamp.secs = 0
        pose.header.stamp.nsecs = 0
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = xGoal
        pose.pose.pose.position.y = yGoal
        pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, 1.57)  
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = orientation_z
        pose.pose.pose.orientation.w = orientation_w
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0,0.0, 0.0, 0.0, covariance]
        rospy.sleep(1)
        self.pub_setpose.publish(pose)
        rospy.loginfo('Published robot pose: %s' % pose)
    
    # move_base
    def moveToGoal(self, xGoal, yGoal, orientation_z, orientation_w):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      
            sys.exit(0)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position =  Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = orientation_z 
        goal.target_pose.pose.orientation.w = orientation_w

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal) 

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
        
    # speed command
    def pub_vel(self, x, y , theta):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = theta
        self.pub.publish(twist)

    # 吸泵控制函数
    def pump_on(self):
        GPIO.output(26, GPIO.LOW)
        GPIO.output(19, GPIO.HIGH)

    def pump_off(self):
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(19, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(19, GPIO.HIGH)

def pick(angle_watch,box_height,pick_times=1):
    global scanner 
    for i in range(pick_times): #i=1,快递盒子一次只吸取一个，先识别一层
        mc.send_angles(angle_watch, 50) # 相机拍照位 
        time.sleep(1)

        while True :
            qr_texts,tvecs =scanner.start_capture() # 获取QR码城市信息和tvec位移矩阵
            time.sleep(1)
            print("qr_texts",qr_texts)
            print("tvecs",tvecs)

            if qr_texts is not None : 
                curr_coords = mc.get_coords() # 获取当前位姿
                print("curr_coords",curr_coords)
                time.sleep(2)
                
                while curr_coords is None:
                    time.sleep(0.5)
                    curr_coords=mc.get_coords()
                    print("coords_s is None")
                    if curr_coords is not None:
                        break

                mat = homo_transform_matrix(*curr_coords) @ homo_transform_matrix(-10, -35, 10, 0, 0, 0)  #矩阵乘积，mat表达了描述机械臂从当前位姿到固定位姿的变换过程的齐次变换矩阵。(-10, -45, 10, 0, 0, 0)为手眼矩阵
                p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1]) # 将列表第一个二维码的tvec位移矩阵转为齐次坐标
                p_base = np.squeeze((mat @ p_end)[:-1]).astype(int) #将转换矩阵与齐次坐标形式的二维码位移向量相乘，得到[x,y,z,1]并去除最后一个元素(齐次坐标)，最后转为整数类型。

                # X error compensation
                p_base[0] +=0
                # Y error compensation
                p_base[1] +=20
                # Z轴固定高度
                p_base[2] = box_height

                new_coords = np.concatenate([p_base, curr_coords[3:]]) # 将x,y,z和当前姿态进行连接成一个新数组
                print("move_coords",list(new_coords))
                mc.send_coords(list(new_coords),20,1)
                time.sleep(2)

                map_navigation.pump_on()
                time.sleep(2)
                print("pump_on")

                curr_coords = mc.get_coords() # 获取当前位姿
                print("curr_coords",curr_coords)
                time.sleep(2)
                
                while curr_coords is None:
                    time.sleep(0.5)
                    curr_coords=mc.get_coords()
                    print("coords_s is None")
                    if curr_coords is not None:
                        break
                
                curr_coords[2]+=40  # z轴抬高
                mc.send_coords(curr_coords,40,mode=1) #z轴抬高
                time.sleep(2)

                mc.send_angles(angle_table["pick_point2"], 50)
                time.sleep(1)

                mc.send_angles(angle_table["place_init"], 50)
                time.sleep(2)

                coords_s = mc.get_coords() #获取当前位姿
                print(coords_s)
                time.sleep(2)
                
                while coords_s is None:
                    time.sleep(0.5)
                    coords_s=mc.get_coords()
                    print("coords_s is None")
                    if coords_s is not None:
                        break

                hight= 45
                coords_s[2]-=hight
                mc.send_coords(coords_s,40,mode=1) #z轴下降
                time.sleep(2)
                map_navigation.pump_off()
                print("pump_off")

                coords_s[2]+=hight        
                mc.send_coords(coords_s,40,mode=1) #z轴抬高
                time.sleep(2)

                mc.send_angles(angle_table["place_point4"], 50) # 过渡点，防止撞掉盒子
                time.sleep(2)

                scanner = None
                scanner = QRCodeScanner()
                break

            else:
                print("qr scanner failed")

    # 结束后复位
    mc.send_angles(angle_table["move_init"], 50)
    time.sleep(1)
    return qr_texts

def load():
    mc.send_angles([0,0,0,0,0,0], 60)
    time.sleep(2)

    mc.send_angles(angle_table["place_init"], 50)
    time.sleep(2)

    coords_s = mc.get_coords() #获取当前位姿
    print(coords_s)
    time.sleep(2)
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]-=70
    mc.send_coords(coords_s,40,mode=1) #z轴下降
    time.sleep(2)
    map_navigation.pump_on()
    time.sleep(2)
    print("pump_off")

    coords_s = mc.get_coords()
    print(coords_s)
    time.sleep(2)
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]+=70
    mc.send_coords(coords_s,40,mode=1) #z轴抬高
    time.sleep(2)

    mc.send_angles(angle_table["place_point4"], 50)
    time.sleep(1)

    mc.send_angles(angle_table["place_point2"], 50)
    time.sleep(2)

    mc.send_angles(angle_table["place_point3"], 50)
    time.sleep(2)
    map_navigation.pump_off()

    # 结束后复位
    mc.send_angles(angle_table["move_init"], 50)
    time.sleep(1)

def ocr_recognized():
        
    # 目标点列表，按照你给定的顺序组织
    goals_sequence = [
        (box_goals_0[0], box_goals_2[0], box_goals_2[1]),  # box_goals_0 1号点 -> box_goals_2 1号、2号点
        (box_goals_0[1], box_goals_2[2], box_goals_2[3]),  # box_goals_0 2号点 -> box_goals_2 3号、4号点
        (box_goals_0[2], box_goals_2[4])                   # box_goals_0 3号点 -> box_goals_2 5号点
    ]

    # 遍历目标点顺序进行导航
    for goal_set in goals_sequence:
        for i, goal in enumerate(goal_set):
            
            # 目标坐标
            x_goal, y_goal, orientation_z, orientation_w = goal
            print(f"导航到目标点: x={x_goal}, y={y_goal}, 方向z={orientation_z}, 方向w={orientation_w}")
            
            # 执行导航
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
            # 根据是否到达目标执行OCR识别
            if flag_feed_goalReached:
                if i > 0:                    
                    recognized_ocr_texts.append(ocr_capture.start_capture())  # 识别文字，存储盒子的变量
            else:
                recognized_ocr_texts.append(None)  # 如果目标未到达，追加None

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    # Close all connections
    running_flag = False
    print("Connections closed.")
    sys.exit()

if __name__ == '__main__':

    box_goals_0 = [
        [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919],#中间一号点,姿态朝前
        [-1.1358978748321533,1.0654418468475342,0.8887916047179362,0.45831155711253446],#中间二号点,姿态朝前      
        [-1.7721543312072754,1.5437819957733154,0.8958855713120555,0.4442848670784004] #中间三号点,姿态朝前
    ]

    box_goals_1 = [
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#中间一号点,姿态朝后
        [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123],#中间一号点,姿态朝后
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#中间二号点,姿态朝后
        [-1.1358978748321533,1.0654418468475342,-0.49958137465689806,0.8662669623712566],#中间二号点,姿态朝后
        [-1.7721543312072754,1.5437819957733154,-0.4625007280446197,0.8866189015344736] #中间三号点,姿态朝后
    ]

    box_goals_2 = [
        [-0.7391788959503174,0.2486436188220978,0.2948542038624959,0.9555422536259784],#1号盒子姿态
        [-0.7391788959503174,0.2486436188220978,-0.953971689949811,0.29989667349655913],#2号盒子姿态
        [-1.1358978748321533,1.0654418468475342,0.29911497291546424,0.9542170785401931],#3号盒子姿态
        [-1.1358978748321533,1.0654418468475342,-0.952682813485717,0.30396621011049635],#4号盒子姿态
        [-1.7721543312072754,1.5437819957733154,-0.9580734245756398,0.28652279686249366]#5号盒子姿态
    ]

    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#中间一号点,姿态朝前
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#中间一号点,姿态朝后
    pack_goal = [-1.7727944374084473,1.6765453577041626,0.3845709618323087,0.9230954313154046]#快递分拣盒附近
    charge_goal =[-1.003286600112915,-0.6634970903396606,0.27711757476357934,0.9608360160595314]

    pack_pose = [-1.5043163299560547,2.357182264328003,0.2875093576208822,0.9577778287684612,0.06853892326654787]

    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch":[94.13, 15.2, -21.88, 0.96, 90.79, 4.57],    #相机拍照位2,中心点快递盒子
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #抓取过渡点
    "place_init":[-93.6, 1.93, 6.24, -0.17, 68.81, -6.24],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],    #放盒子点位
    "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, 1.46]
    }

    city_to_region_mapping = {
        '北京市': '华北区',
        '上海市': '华东区',
        '南京市': '华东区',
        '东莞市': '华南区',
        '广州市': '华南区',
        '武汉市': '华中区',
        '大连市': '东北区',
    }

    initialized = True # 导航初始动作
    box_2_height = 101  #快递盒子第二层Z轴高度101,demo2 140
    box_1_height = 60   #快递盒子第一层Z轴高度60,demo2 90

    boxes_with_text = []
    recognized_ocr_texts = ['华东区','华南区','华北区','华中区','东北区'] #固定快递分拣点
    recognized_ocr = [] # ocr识别添加快递分拣点
    recognized_qr_texts = []

    map_navigation = MapNavigation()
    ocr_capture = OCRVideoCapture()
    # global scanner
    scanner = QRCodeScanner()

    plist = get_port_list()
    print(plist)
    mc = MechArm270(plist[0],115200) # 连接机械臂

    mc.send_angles(angle_table["move_init"], 50)

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    ##########################################################
    # # 功能一：记录五个导航点的信息
    ##########################################################
    # ocr_recognized() # 视觉识别误差较大,该功能暂不使用

    for text, box_goals_1, box_goals_2 in zip(recognized_ocr_texts, box_goals_1, box_goals_2):
        box_info = {
            "text": text,
            "box_goals_1": box_goals_1,
            "box_goals_2": box_goals_2,
        }
        boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print(box)
        
    ##########################################################
    # # 功能二：循环5次,每次只抓一个盒子,然后对盒子进行分拣
    ##########################################################
    for i in range(5):
        if (initialized):
            initialized = False
            x_goal, y_goal, orientation_z, orientation_w = goal_1
            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
        x_goal, y_goal, orientation_z, orientation_w = pack_goal
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:

            print("python agv_aruco")
            os.system('python agv_aruco.py') 

            xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
            map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) #amcl重定位

            # 根据循环次数抓取,固定相机拍照位和吸取高度
            if i == 0:
                angle_pick = angle_table["pick_watch"]
                box_height = box_2_height
            elif i == 1:
                angle_pick = angle_table["pick_watch"]
                box_height = box_1_height
            elif i == 2:
                angle_pick = angle_table["pick_watch"]
                box_height = box_2_height
            elif i == 3:
                angle_pick = angle_table["pick_watch"]
                box_height = box_1_height
            elif i == 4:
                angle_pick = angle_table["pick_watch"]
                box_height = box_1_height
        
            recognized_qr_texts.append(pick(angle_pick,box_height))  # 发送拍照相机关节角度和Z轴高度，抓取并返回识别文字

            # x平移1秒
            map_navigation.pub_vel(-0.1,0,0)
            time.sleep(2.5)
            map_navigation.pub_vel(0,0,0)

            # 旋转180°
            map_navigation.pub_vel(0,0,-0.1)
            time.sleep(4)
            map_navigation.pub_vel(0,0,0)

            # x平移1秒
            map_navigation.pub_vel(0.1,0,0)
            time.sleep(2)

        else:
            print("failed")

    ##########################################################
    # # 功能三：遍历识别到的所有市级名称，依次导航
    ##########################################################
        if recognized_qr_texts: #recognized_qr_texts = ['上海市', '南京市','武汉市','北京市','大连市']  # 通过 OCR 获取的市级列表
            print("recognized_ocr_texts:",recognized_ocr_texts) #debug
            print("recognized_qr_texts:",recognized_qr_texts)   #debug 
            # 获取列表中的最后一个城市
            last_city = recognized_qr_texts[-1]
            region = city_to_region_mapping.get(last_city, "未知区域") # 上海市：华东区，返回华东区
            if region != "未知区域":
                # 查找与该区域对应的目标位置
                for box in boxes_with_text:
                    if box["text"] == region:

                        # 获取该区域的两个个目标点
                        box_goals_1 = box["box_goals_1"]
                        box_goals_2 = box["box_goals_2"]

                        # 遍历目标点和方向信息，依次导航到每个目标
                        for target_num, goal in enumerate([box_goals_1, box_goals_2], 1):
                            # 目标坐标
                            x_goal, y_goal, orientation_z, orientation_w = goal

                            print(f"导航到{region}的目标{target_num}: x={x_goal}, y={y_goal}, 方向z={orientation_z}, 方向w={orientation_w}")
                            map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        recognized_ocr.append(ocr_capture.start_capture())

                        print(f"识别到{recognized_ocr[-1]},{region}快递即将搬运至{recognized_ocr[-1]}")

                        print("python agv_aruco")
                        os.system('python agv_aruco.py')    # 导航目标点
                        
                        load()  #导航完所有点进行放盒子
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(5.7)
                        map_navigation.pub_vel(0,0,0)
      
    ##########################################################
    # # 功能四：回充套装 充电区离墙太近了，里程计误差，导航容易卡在该点位
    ##########################################################

    x_goal, y_goal, orientation_z, orientation_w = goal_1_back #先导航到该点，避免撞到快递放置盒
    flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
    if flag_feed_goalReached:

        x_goal, y_goal, orientation_z, orientation_w = charge_goal
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            pass #回充套装

            # y平移2秒
            # map_navigation.pub_vel(0,-0.1,0)
            # time.sleep(2)
            # map_navigation.pub_vel(0,0,0)  
            # x平移2秒
            map_navigation.pub_vel(-0.1,0,0)
            time.sleep(2)
            map_navigation.pub_vel(0,0,0)

        else:
            print("failed")
        
