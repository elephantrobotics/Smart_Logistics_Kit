#!/usr/bin/env python                                                                                                                      
#coding=UTF-8
import rospy
import time
import actionlib
import signal
import sys
import Jetson.GPIO as GPIO

from pymycobot.mycobot import MyCobot
from pymycobot.utils import get_port_list

from OCRVideoCapture import OCRVideoCapture
from QRCodeScanner import QRCodeScanner

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

    def pump_on(self):
        GPIO.output(26, GPIO.LOW)
        GPIO.output(19, GPIO.HIGH)

    def pump_off(self):
        GPIO.output(26, GPIO.HIGH)
        GPIO.output(19, GPIO.LOW)
        time.sleep(0.05)
        GPIO.output(19, GPIO.HIGH)

def pick():
    global scanner
    mc.send_angles([0,0,0,0,0,0], 60)
    time.sleep(2)
    for i in range(1):
        mc.send_angles(angle_table["pick_watch"], 50)    

        time.sleep(1)
        print(scanner.start_capture())
        time.sleep(5)   

        mc.send_angles(angle_table["pick_init"], 50)    
        time.sleep(1)

        coords_s = mc.get_coords()
        print(coords_s)
        time.sleep(2)
        
        while coords_s is None:
            time.sleep(0.5)
            coords_s=mc.get_coords()
            print("coords_s is None")
            if coords_s is not None:
                break
        
        if i ==0:
            hight=40
        else:
            hight=65

        coords_s[2]-=hight
        mc.send_coords(coords_s,40,mode=1) #z轴下降
        time.sleep(2)
        map_navigation.pump_on()
        time.sleep(2)
        print("pump_on")

        coords_s[2]+=hight        
        mc.send_coords(coords_s,40,mode=1) #z轴抬高
        time.sleep(2)
########################################################################################rount2
        mc.send_angles(angle_table["pick_point2"], 50)
        time.sleep(1)

        mc.send_angles(angle_table["place_init"], 50)
        time.sleep(2)

        coords_s = mc.get_coords()
        print(coords_s)
        time.sleep(2)
        
        while coords_s is None:
            time.sleep(0.5)
            coords_s=mc.get_coords()
            print("coords_s is None")
            if coords_s is not None:
                break

        if i ==0:
            hight=20
        else:
            hight=15

        coords_s[2]-=hight
        mc.send_coords(coords_s,40,mode=1) #z轴下降
        time.sleep(2)
        map_navigation.pump_off()
        print("pump_off")

        coords_s[2]+=hight        
        mc.send_coords(coords_s,40,mode=1) #z轴抬高
        time.sleep(2)

        scanner = None
        scanner = QRCodeScanner()

    # 结束后复位
    print(area_table)
    mc.send_angles(angle_table["move_init"], 50)
    time.sleep(1)

def load():
    mc.send_angles([0,0,0,0,0,0], 60)
    time.sleep(2)

    mc.send_angles(angle_table["place_init"], 50)
    time.sleep(2)

    coords_s = mc.get_coords()
    print(coords_s)
    time.sleep(2)
    
    while coords_s is None:
        time.sleep(0.5)
        coords_s=mc.get_coords()
        print("coords_s is None")
        if coords_s is not None:
            break

    coords_s[2]-=50
    mc.send_coords(coords_s,40,mode=1) #z轴下降
    time.sleep(2)
    map_navigation.pump_on()
    time.sleep(2)
    print("pump_off")

    mc.send_angles(angle_table["place_point4"], 50)
    time.sleep(2)

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
   
    goal_1 = [0.4281424582004547,0.6189473509788513,-0.018068829690399985,0.9998367453707727]
    goal_2 = [0.8920876979827881,0.6039064764976501,0.00469590534314955,0.9999889741757196]
    goal_3 = [1.2844168424606323,0.620277214050293,0.03193667903827325,0.9994898941620202]
    pack_goal = [1.4083706140518188,0.49778820276260376,0.00014766695622705518,0.999999989097235]
    charge_goal =[-0.027686625719070435,0.9285135269165039,-0.6785262706093567,0.7345761363486824]

    pose_1 = [0.4281424582004547,0.7189473509788513,-0.018068829690399985,0.9998367453707727,0.06853892326654787]
    pose_2 = [0.8920876979827881,0.7039064764976501,0.00469590534314955,0.9999889741757196,0.06853892326654787]
    pose_3 = [1.4844168424606323,0.820277214050293,0.03193667903827325,0.9994898941620202,0.06853892326654787]
    pack_pose = [1.7001869678497314,0.28472626209259033,0.009679434542179345,0.9999531531761594,0.06853892326654787]
    charge_pose = [-0.027686625719070435,0.9285135269165039,-0.6785262706093567,0.7345761363486824,0.06853892326654787]
    
    angle_table = {
    "zero_position":[0,0,0,0,0,0],
    "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_watch":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_init":[1.05, 38.23, -39.99, -1.05, 86.48, -6.5],
    "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19], #抓取过渡点
    "place_init":[-95.71, 22.41, -25.04, -3.07, 95.27, 1.4],
    "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, -23.46],
    "place_point2":[-6.85, 24.78, -20.03, -2.9, 81.03, 11.16],
    "place_point3":[84.99, 50.62, -45.17, -2.37, 79.54, 11.42]
    }

    area_table = {
    "pack_0":[],
    "pack_1":[],
    }

    map_navigation = MapNavigation()

    # ocr_capture = OCRVideoCapture(camera_index=0)
    global scanner
    scanner = QRCodeScanner()

    plist = get_port_list()
    print(plist)
    mc = MyCobot(plist[0],115200) # 连接机械臂

    mc.send_angles(angle_table["move_init"], 50)

    # print(mc.get_angles())

    # print(ocr_capture.start_capture()) # 识别文字，打印一号盒子的变量

    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)

    # while running_flag:
    #     break
    for i in range(1):##demo

        x_goal, y_goal, orientation_z, orientation_w = goal_1
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            xGoal, yGoal, orientation_z, orientation_w,covariance = pose_1
            #map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance)

            # 旋转90°
            map_navigation.pub_vel(0,0,-0.1)
            time.sleep(3.5)
            map_navigation.pub_vel(0,0,0)
            print("rount 1")

            time.sleep(3)
            pass# print(ocr_capture.start_capture()) # 识别文字，打印一号盒子的变量

            # 旋转180°
            map_navigation.pub_vel(0,0,-0.1)
            time.sleep(8)
            map_navigation.pub_vel(0,0,0)
            print("rount 2")

            time.sleep(3)
            pass # 识别文字，记下二号盒子的变量
        else:
            print("failed")

        x_goal, y_goal, orientation_z, orientation_w = goal_2
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            xGoal, yGoal, orientation_z, orientation_w,covariance = pose_2
            #map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance)
            # 旋转90°
            map_navigation.pub_vel(0,0,-0.1)
            time.sleep(3.5)
            map_navigation.pub_vel(0,0,0)
            print("rount 1")

            time.sleep(3)
            pass# print(ocr_capture.start_capture()) # 识别文字，打印三号盒子的变量

            # 旋转180°
            map_navigation.pub_vel(0,0,-0.1)
            time.sleep(8)
            map_navigation.pub_vel(0,0,0)
            print("rount 2")

            time.sleep(3)
            pass # 识别文字，记下四号盒子的变量
        else:
            print("failed")

        x_goal, y_goal, orientation_z, orientation_w = goal_3
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            xGoal, yGoal, orientation_z, orientation_w,covariance = pose_3
            #map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance)
            # 旋转90°
            map_navigation.pub_vel(0,0,0.1)
            time.sleep(3.5)
            map_navigation.pub_vel(0,0,0)
            print("rount 1")
            time.sleep(3)
            # print(ocr_capture.start_capture()) # 识别文字，打印五号盒子的变量
        else:
            print("failed")


    # ########################################################
    # # 货架区域离墙太近了，里程计误差，导航容易卡在该点位
        x_goal, y_goal, orientation_z, orientation_w = pack_goal
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            xGoal, yGoal, orientation_z, orientation_w,covariance = pack_pose
            map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance)

            # y平移两秒
            map_navigation.pub_vel(0,-0.1,0)
            time.sleep(2.35)
            map_navigation.pub_vel(0,0,0)

            # y平移两秒
            map_navigation.pub_vel(0.1,0,0)
            time.sleep(1)

            map_navigation.pub_vel(0,0,0) 

            pass    # 识别       
            pick()  # 抓取      
        else:
            print("failed")
    # # #######################################################
    # 导航到2号区域的4号盒子
        x_goal, y_goal, orientation_z, orientation_w = goal_2
        flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        if flag_feed_goalReached:
            xGoal, yGoal, orientation_z, orientation_w,covariance = pose_2
            # 旋转90°
            map_navigation.pub_vel(0,0,0.1)
            time.sleep(3.5)
            map_navigation.pub_vel(0.1,0,0)
            time.sleep(5.7)
            map_navigation.pub_vel(0,0,0)
            print("rount 1")
            
            load()

            map_navigation.pub_vel(-0.1,0,0)
            time.sleep(5.7)
            map_navigation.pub_vel(0,0,0)

    # #######################################################
    # # 导航到1号区域的1号盒子
    #     x_goal, y_goal, orientation_z, orientation_w = goal_1
    #     flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
    #     if flag_feed_goalReached:
    #         xGoal, yGoal, orientation_z, orientation_w,covariance = pose_1
    #         # 旋转90°
    #         map_navigation.pub_vel(0,0,-0.1)
    #         time.sleep(3.5)
    #         map_navigation.pub_vel(0.1,0,0)
    #         time.sleep(1.5)
    #         map_navigation.pub_vel(0,0,0)
    #         print("rount 1")
            
    #         load()

    #         map_navigation.pub_vel(-0.1,0,0)
    #         time.sleep(1.5)
            
    # #######################################################
    # # 充电区离墙太近了，里程计误差，导航容易卡在该点位

    #     x_goal, y_goal, orientation_z, orientation_w = charge_goal
    #     flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
    #     if flag_feed_goalReached:
    #         xGoal, yGoal, orientation_z, orientation_w,covariance = charge_goal
    #         map_navigation.set_pose(xGoal, yGoal, orientation_z, orientation_w,covariance)
    #     else:
    #         print("failed")