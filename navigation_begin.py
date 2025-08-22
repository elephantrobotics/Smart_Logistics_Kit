import rospy
import time
import actionlib
import signal
import sys
import os
import numpy as np

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
            
def signal_handler(signal, frame):
    print("Ctrl+C pressed. Exiting...")
    # Close all connections
    running_flag = False
    print("Connections closed.")
    sys.exit()


if __name__ == '__main__':
    initialized = True # 导航初始动作
    map_navigation = MapNavigation()
    
    goal_1 = [-0.7349843764305115,0.24553439617156982,0.8816407909804326,0.471921090521919]#中间一号点,姿态朝前
    goal_1_back = [-0.7391788959503174,0.2486436188220978,-0.468811666883722,0.8832981495473123]#中间一号点,姿态朝后
    pack_goal = [-1.6927944374084473,1.6765453577041626,0.29911497291546424,0.9542170785401931]#快递分拣盒附近
    
    # Register the Ctrl+C signal handler
    global running_flag 
    running_flag = True
    signal.signal(signal.SIGINT, signal_handler)
    
    if (initialized):
        initialized = False
        x_goal, y_goal, orientation_z, orientation_w = goal_1
        map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
        print(f"导航到目标点: x={x_goal}, y={y_goal}, 方向z={orientation_z}, 方向w={orientation_w}")
    
    # x_goal, y_goal, orientation_z, orientation_w = goal_1
    # map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
            
    x_goal, y_goal, orientation_z, orientation_w = pack_goal
    flag_feed_goalReached = map_navigation.moveToGoal(x_goal, y_goal, orientation_z, orientation_w)
    print(f"导航到目标点: x={x_goal}, y={y_goal}, 方向z={orientation_z}, 方向w={orientation_w}")
    
