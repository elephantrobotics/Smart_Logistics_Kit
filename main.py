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

from pymycobot.mycobot import MyCobot
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
        mc.send_angles([0,0,0,0,0,0], 60)
        time.sleep(1)
        mc.send_angles(angle_table["pick_init"], 50) 
        time.sleep(1)

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
                p_base[1] +=0
                # Z轴固定高度
                p_base[2] = box_height

                new_coords = np.concatenate([p_base, curr_coords[3:]]) # 将x,y,z和当前姿态进行连接成一个新数组
                print("move_coords",new_coords)
                mc.send_coords(new_coords,50)
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

    coords_s[2]-=90
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

    coords_s[2]+=90
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

def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    # Close all connections
    running_flag = False
    print("Connections closed.")
    sys.exit()

if __name__ == '__main__':
   
    box_goals_1 = [
        [0.4281424582004547,0.6189473509788513,-0.9997609919918943,0.021862270956683708],#中间一号点
        [0.4281424582004547,0.6189473509788513,-0.9997609919918943,0.021862270956683708],#中间一号点
        [0.8920876979827881,0.6039064764976501,-0.9997609919918943,0.021862270956683708],#中间二号点
        [0.8920876979827881,0.6039064764976501,-0.9997609919918943,0.021862270956683708],#中间二号点
        [1.2844168424606323,0.620277214050293,-0.9997609919918943,0.0218622709566837083] #中间三号点
    ]

    box_goals_2 = [
        [0.4281424582004547,0.6189473509788513, -0.7108462641581993, 0.7033474168093117],#1号盒子姿态
        [0.4281424582004547,0.6189473509788513, 0.7681211233139038, 0.7228808403015137],#2号盒子姿态
        [0.8920876979827881, 0.6039064764976501, -0.7108462641581993, 0.7033474168093117],#3号盒子姿态
        [0.8920876979827881, 0.6039064764976501, 0.7681211233139038, 0.7228808403015137],#4号盒子姿态
        [1.2844168424606323,0.620277214050293, 0.7681211233139038, 0.7228808403015137]#5号盒子姿态
    ]

    goal_1 = [0.4281424582004547,0.6189473509788513,-0.018068829690399985,0.9998367453707727]#中间一号点
    goal_2 = [0.8920876979827881,0.6039064764976501,0.00469590534314955,0.9999889741757196]#中间二号点
    goal_3 = [1.2844168424606323,0.620277214050293,0.03193667903827325,0.9994898941620202]#中间三号点
    pack_goal = [1.2083706140518188,0.29778820276260376,0.00014766695622705518,0.999999989097235]
    charge_goal =[-0.027686625719070435,0.9285135269165039,-0.6785262706093567,0.7345761363486824]

    pack_pose = [1.7001869678497314,0.28472626209259033,0.009679434542179345,0.9999531531761594,0.06853892326654787]
    charge_pose = [-0.027686625719070435,0.9285135269165039,-0.6785262706093567,0.7345761363486824,0.06853892326654787]
    
    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch1":[-38.05, 43.41, -34.54, 0.87, 71.71, -36.29],  #相机拍照位1,右侧的快递盒子
    "pick_watch2":[1.05, 38.23, -39.99, -1.05, 86.48, -6.5],    #相机拍照位2,中心点快递盒子
    "pick_watch3":[14.06, 46.23, -31.46, -3.25, 71.54, 13.27],   #相机拍照位3,左侧的快递盒子
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],    #抓取过渡点
    "place_init":[-95.71, 22.41, -25.04, -3.07, 95.27, 1.4],
    "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3":[84.99, 50.62, -45.17, -2.37, 79.54, 11.42],
    "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, 1.46]
    }

    city_to_region_mapping = {
        '北京市': '华北区',
        '上海市': '华东区',
        '南京市': '华东区',
        '东莞市': '华南区',
        '武汉市': '华中区',
        '大连市': '东北区',
    }

    boxes_with_text = []
    recognized_ocr_texts = ['华东区','华南区','华北区','华中区','东北区']
    recognized_qr_texts = []

    map_navigation = MapNavigation()
    ocr_capture = OCRVideoCapture()
    # global scanner
    scanner = QRCodeScanner()

    plist = get_port_list()
    print(plist)
    mc = MyCobot(plist[0],115200) # 连接机械臂

    mc.send_angles(angle_table["move_init"], 50)

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    # while running_flag:
    #     break

    # # #######################################################记录五个导航点的信息
    for text, box_goals_1, box_goals_2 in zip(recognized_ocr_texts, box_goals_1, box_goals_2):
        box_info = {
            "text": text,
            "box_goals_1": box_goals_1,
            "box_goals_2": box_goals_2,
        }
        boxes_with_text.append(box_info)
    
    for box in boxes_with_text:
        print(box)
        
    # ########################################################
    for i in range(5):##demo 循环5次,每次只抓一个盒子,然后对盒子进行分拣
        x_goal, y_goal, orientation_z, orientation_w = goal_1 #先导航到该点，避免撞到快递放置盒
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            x_goal, y_goal, orientation_z, orientation_w = pack_goal
            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            if flag_feed_goalReached:

                # y平移2秒
                map_navigation.pub_vel(0,-0.1,0)
                time.sleep(2)
                map_navigation.pub_vel(0,0,0)

                print("python agv_aruco")
                os.system('python agv_aruco.py') 

                xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
                map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance) #amcl重定位

                # 根据循环次数抓取,固定相机拍照位和吸取高度
                if i == 0:
                    angle_pick = angle_table["pick_watch1"]
                    box_height = 61
                elif i == 1:
                    angle_pick = angle_table["pick_watch2"]
                    box_height = 101
                elif i == 2:
                    angle_pick = angle_table["pick_watch2"]
                    box_height = 61
                elif i == 3:
                    angle_pick = angle_table["pick_watch3"]
                    box_height = 101
                elif i == 4:
                    angle_pick = angle_table["pick_watch3"]
                    box_height = 61
              
                recognized_qr_texts.append(pick(angle_pick,box_height))  # 发送拍照相机关节角度和Z轴高度，抓取并返回识别文字

                # x平移1秒
                map_navigation.pub_vel(-0.1,0,0)
                time.sleep(1)

                # y平移2秒
                map_navigation.pub_vel(0,0.1,0)
                time.sleep(2.35)
                map_navigation.pub_vel(0,0,0)

                # 旋转180°
                map_navigation.pub_vel(0,0,-0.1)
                time.sleep(8)
                map_navigation.pub_vel(0,0,0)

                # x平移1秒
                map_navigation.pub_vel(0.1,0,0)
                time.sleep(2)

            else:
                print("failed")

    # # #######################################################
    # 遍历识别到的所有市级名称，依次导航
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
                            flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)

                        print("python agv_aruco")
                        os.system('python agv_aruco.py')    # 导航目标点
                        
                        load()  #导航完所有点进行放盒子
                        map_navigation.pub_vel(-0.1,0,0)
                        time.sleep(5.7)
                        map_navigation.pub_vel(0,0,0)
      
    # #######################################################
    # # 充电区离墙太近了，里程计误差，导航容易卡在该点位
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
        time.sleep(3.5)
        map_navigation.pub_vel(0,0,0)

    else:
        print("failed")
        
