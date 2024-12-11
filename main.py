#!/usr/bin/env python                                                                                                                      
#coding=UTF-8
import rospy
import time
import actionlib

from pymycobot.mycobot import MyCobot
from pymycobot.utils import get_port_list

from OCRVideoCapture import OCRVideoCapture

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


if __name__ == '__main__':
   
    goal_1 = [(-1.8894099950790405,0.6915320158004761,-0.7476189288897872,0.6641279524050221)]
    goal_2 = [(-1.1645034551620483,1.5066015243530273,-0.03527231359038596,0.9993777383422053)]
    goal_3 = [(-1.7634624004364014,0.6029707193374634,-0.7010442389772473,0.7131177847991257)]

    map_navigation = MapNavigation()

    ocr_capture = OCRVideoCapture(camera_index=0) 

    plist = get_port_list()
    print(plist)
    mc = MyCobot(plist[0],115200) # 连接机械臂

    x_goal, y_goal, orientation_z, orientation_w = goal_1
    flag_feed_goalReached = map_navigation.navigate(x_goal, y_goal, orientation_z, orientation_w)
    if flag_feed_goalReached:
        map_navigation.set_pose(-1.8611218929290771,0.028858069330453873,-0.6969873407167377,0.7170834309064812,0.06853892326654787)
        pass # 旋转90°
        print(ocr_capture.start_capture()) # 识别文字，打印一号盒子的变量
        pass # 旋转90°
        pass # 识别文字，记下二号盒子的变量
    else:
        print("failed")

    x_goal, y_goal, orientation_z, orientation_w = goal_2
    flag_feed_goalReached = map_navigation.navigate(x_goal, y_goal, orientation_z, orientation_w)
    if flag_feed_goalReached:
        map_navigation.set_pose(-1.8611218929290771,0.028858069330453873,-0.6969873407167377,0.7170834309064812,0.06853892326654787)
    else:
        print("failed")

    x_goal, y_goal, orientation_z, orientation_w = goal_3
    flag_feed_goalReached = map_navigation.navigate(x_goal, y_goal, orientation_z, orientation_w)
    if flag_feed_goalReached:
        map_navigation.set_pose(-1.8611218929290771,0.028858069330453873,-0.6969873407167377,0.7170834309064812,0.06853892326654787)
    else:
        print("failed")