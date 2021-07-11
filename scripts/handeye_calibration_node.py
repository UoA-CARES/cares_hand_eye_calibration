#!/usr/bin/env python3
import rospy
import roslib
import math 
import time
from datetime import datetime

import yaml

from math import pi

import numpy as np

import cv2
from cv_bridge import CvBridge

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from cares_msgs.srv import CalibrationService, ArucoDetect

from handeye_calibrator import HandeyeCalibrator, StereoCalibrator, DepthCalibrator

import actionlib
from platform_msgs.msg import PlatformGoalAction, PlatformGoalGoal

import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory

from os.path import expanduser
home = expanduser("~")

def collect_data_samples(filepath, sensor_calibrator):    
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #As we are moving the arm around without a calibrated sensor we use the ee_link to move the arm
    ee_frame = rospy.get_param('~robot_effector_frame')

    #Global link to align data too
    #TODO extract this out via params
    world_link = "world"

    # Creates the SimpleActionClient, passing the type of the action
    platform_server = rospy.get_param('~platform_server', 'server')
    platform_client = actionlib.SimpleActionClient(platform_server, PlatformGoalAction)
    
    print("Waiting for platform server - "+platform_server)
    platform_client.wait_for_server()
    print("Server Ready moving to calibration")

    #TODO extract this out via params
    d_quaternion = quaternion_from_euler(0, 0, 0)
    init_pose = utils.create_pose_msg(0, 0.75, 0.8, quaternion=d_quaternion)
    init_goal = utils.create_goal_msg(init_pose, 0, ee_frame)

    print("Sending init position", init_goal)
    platform_client.send_goal(init_goal)
    platform_client.wait_for_result()
    print("Moved to initial position moving to mapping phase")

    path_factory = PathFactory()
    
    #TODO pull this variable for path selection out...
    pathway = path_factory.create_path(2)
    total = len(pathway)
    for count, pose in enumerate(pathway):
        pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
        
        print("Moving too: "+str(count)+"/"+str(total))
        print(pose_goal.target_pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()
        
        #handle result feedback - fail, success, etc
        result = platform_client.get_state()
        print("Result is ", result)

        if result == 3:# SUCCEEDED
            #########################LOGIC FOR TAKING IMAGES#################################
            time.sleep(1.0)#just to allow the shaking to stop
            file_name = filepath+str(count)
            sensor_calibrator.collect_samples(file_name, tf_buffer)
        else:
            print("Failed to reach pose")

        if rospy.is_shutdown():
            return
                            
    print("moving back to home position")
    platform_client.send_goal(init_goal)

def generate_filepath():
    now = datetime.now()
    now = now.strftime("%Y-%m-%d-%H-%M-%S")
    filepath = home +"/calibration_images/"+now+"/"
    import os
    if not os.path.exists(filepath):
        os.makedirs(filepath)
    return filepath

def main():
    rospy.init_node('stereo_auto_calibration_node')

    sensor = rospy.get_param('~sensor') # 'depth' 'stereo'
    if sensor == 'depth':
        sensor_calibrator = DepthCalibrator()
    elif sensor == 'stereo':
        sensor_calibrator = StereoCalibrator()
    else:
        print("Sensor not defined: "+str(sensor))
        return

    filepath = ""
    filepath = rospy.get_param('~file_path', filepath)
    image_list = []
    if filepath != "":
        sensor_calibrator.load_data(filepath)
    else:
        filepath = generate_filepath()
        collect_data_samples(filepath, sensor_calibrator)

    print("Saving data too: "+filepath)

    sensor_calibrator.calibrate(filepath)
    
    print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
