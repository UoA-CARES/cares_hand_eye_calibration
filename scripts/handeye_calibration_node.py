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

from handeye_calibrator import HandeyeCalibrator, StereoCalibrator, DepthCalibrator, CalibratorFactory

import actionlib
from platform_msgs.msg import PlatformGoalAction
from actionlib_msgs.msg import GoalStatus

import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory, World

from os.path import expanduser
home = expanduser("~")



messages = {result : key for key, result in GoalStatus.__dict__.items() 
    if not key.startswith("_") and isinstance(result, int)}



def move_path(platform_client, pathway, ee_frame):
    for pose in pathway:
        pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
        
        rospy.loginfo(pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()
        
        #handle result feedback - fail, success, etc
        result = platform_client.get_state()
        rospy.loginfo(f"move_path: {result} {messages[result]}")

        yield (pose, result)

        if rospy.is_shutdown():
            return
                            


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
    # d_quaternion = quaternion_from_euler(0, 0, 0)
    # init_pose = utils.create_pose_msg(0, 0.75, 0.8, quaternion=d_quaternion)


    init_goal = utils.create_goal_msg( PathFactory.home_pose, 0, ee_frame)

    print("Sending init position", init_goal)
    platform_client.send_goal(init_goal)
    platform_client.wait_for_result()
    print("Moved to initial position moving to mapping phase")


    pathway = PathFactory.calibration()
    total = len(pathway)
    for count, pose in enumerate(pathway):
        pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
        
        print("Moving to: "+str(count)+"/"+str(total))
        rospy.loginfo(pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()
        
        #handle result feedback - fail, success, etc
        result = platform_client.get_state()
        print("Result is ", result)

        if result == GoalStatus.SUCCEEDED:
            time.sleep(1.0)#just to allow the shaking to stop
            file_name = filepath+str(count)
            rospy.loginfo(f"Saving to {file_name}")
            sensor_calibrator.collect_samples(file_name, world_link, ee_frame, tf_buffer)
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
    sensor_calibrator = CalibratorFactory.create_calibrator(sensor)

    if sensor_calibrator is None:
        rospy.logerr("unknown Sensor Type: "+str(sensor))
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
