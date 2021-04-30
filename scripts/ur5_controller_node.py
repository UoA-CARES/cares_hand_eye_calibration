#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import tf
import math 

from cv_bridge import CvBridge

from maara_msgs.msg import KeyPoints, ArmCmd
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker

from move_group_python_interface import MoveGroupPythonInterface
from tf.transformations import quaternion_from_euler
from math import pi

import numpy as np
import cv2

import matplotlib.pyplot as plt

from Queue import Queue
from Queue import Empty

import serial

from os.path import expanduser
home = expanduser("~")

# Arm command
# Enumeration
# 0 Move
# 1 Stop
# 2 Reset
# 3 Open Gripper
# 4 Close Gripper
MOVE=0
STOP=1
RESET=2
OPEN=3
CLOSE=4
ENABLE_TASK=5

queue = Queue(maxsize=0)

def callback(arm_command):
    queue.put(arm_command)

def main():
    rospy.init_node('move_group_python_interface', anonymous=True)
    ur5 = MoveGroupPythonInterface()

    cx = rospy.get_param('~cx')
    cy = rospy.get_param('~cy')
    cz = rospy.get_param('~cz')
    dx = rospy.get_param('~dx')
    dy = rospy.get_param('~dy')
    dz = rospy.get_param('~dz')
    
    ur5.add_exlusion_zone(cx, cy, cz, dx, dy, dz, "exclusion_zone")
    ur5.add_exlusion_zone(0, 0, -0.5, 2, 2, 0.01, "ground")
    ur5.add_exlusion_zone(0, -0.4, 0, 2.0, 0.05, 2.0, "backside_protect")
    
    command_subscriber = rospy.Subscriber("arm_command", ArmCmd, callback)

    # ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port

    while not rospy.is_shutdown():
        try:     
            arm_command = queue.get(timeout=0.1)
            #Should this be blocking or not?
            #I.e. should other pose commands override an action currently in progress?
            #Ultimatly extend this to handle commands such as stop/cancel etc?
            cmd = arm_command.command

            if cmd == MOVE:
                ur5.set_ee_link(arm_command.ee_id.data)
                ur5.go_to_pose_goal(arm_command.target_pose)
            elif cmd == OPEN:
                print("Opened Gripper")
                # ser.write(str(0))
            elif cmd == CLOSE:
                print("Closed Gripper")
                # ser.write(str(1))
            elif cmd == ENABLE_TASK:
                print("Enabling Pick")
                ur5.remove_exclusion_zone("exclusion_zone")

        except Empty:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
