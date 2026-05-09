#!/usr/bin/env python
#coding=UTF-8
import os
import timer
import rospy
import time
import actionlib
import signal
import sys
import numpy as np
import Jetson.GPIO as GPIO
import glob
import cv2
import cv2.aruco as aruco
import numpy as np
import http
from pymycobot.mecharm270 import MechArm270
from pymycobot.utils import get_port_list

from OCRVideoCapture import OCRVideoCapture
from QRCodeScanner import QRCodeScanner
from Transformation import homo_transform_matrix
from wit_usb2can import SerialCANParser
from MapNavigation import MapNavigation
from actionlib_msgs.msg import *
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import agv_aruco_1
import xmlrpc
import aruco_detector
class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout=300, use_datetime=0):
        self.timeout = timeout
        super().__init__(use_datetime)

    def make_connection(self, host):
        # This forces the timeout on the underlying HTTP connection
        conn = http.client.HTTPConnection(host, timeout=self.timeout)
        return conn
def get_rpc_proxy(url="http://localhost:6666/", timeout=30):
    proxy = xmlrpc.client.ServerProxy(url, transport=TimeoutTransport(timeout=300))
    start_time = time.time()
    
    print("Waiting for ArUco RPC Server to wake up...")
    while time.time() - start_time < timeout:
        try:
            # We call a dummy system method to check if the server is alive
            proxy.system.listMethods() 
            print("Connected to RPC Server successfully!")
            return proxy
        except (ConnectionRefusedError, OSError):
            time.sleep(1) # Wait 1 second before retrying
            
    raise Exception(f"Could not connect to RPC server after {timeout} seconds.")
def safe_aruco_command(proxy, command, timeout_recovery=True):
    """
    Wraps the RPC call with timeout handling and status polling.
    Returns the server response OR 'Success (Recovered)'
    """
    print(f"--- Sending Command: {command} ---")
    
    try:
        # 1. Attempt the blocking call
        response = proxy.aruco_rpc(command)
        return response

    except socket.timeout:
        if not timeout_recovery:
            raise # Re-raise if we don't want to recover
            
        print(f"(!) Warning: Command '{command}' timed out (Network layer).")
        print("    The robot is likely still working. Switching to status monitoring...")

        # 2. Recovery Loop: Poll until robot is IDLE
        while True:
            try:
                # You MUST have 'get_robot_state' on the server for this to work
                status = proxy.get_robot_state()
                print(f"    -> Robot Status: {status}")
                
                if status == "IDLE" or status == "DONE":
                    print("    -> Robot finished task.")
                    return "Success (Recovered)"
                
                if "ERROR" in status:
                     return f"Robot Error: {status}"

            except Exception as e:
                print(f"    -> Polling failed ({e}), retrying...")
            
            time.sleep(2) # Don't flood the network
            
    except Exception as e:
        print(f"CRITICAL ERROR: RPC Failed completely: {e}")
        return None
if __name__ == '__main__':

    parser = SerialCANParser('/dev/ttyUSB0', 9600, 1)

    print(get_port_list())

    mc = MechArm270('/dev/ttyACM0',115200) 
    mc.set_fresh_mode(0)
    while True:
        aruco_detector.getArucoCode()