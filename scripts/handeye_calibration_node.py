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
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal

import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory

from os.path import expanduser
home = expanduser("~")

def mark_on_rviz(pub, points, world_link, count, success = True):
    # create a grey box marker
    box_marker = Marker()
    box_marker.header.frame_id = world_link
    box_marker.id = count
    box_marker.type = Marker.CUBE
    box_marker.action = box_marker.ADD
    box_marker.pose.position.x = points.pose.position.x
    box_marker.pose.position.y = points.pose.position.y
    box_marker.pose.position.z = points.pose.position.z
    box_marker.pose.orientation.x = points.pose.orientation.x
    box_marker.pose.orientation.y = points.pose.orientation.y
    box_marker.pose.orientation.z = points.pose.orientation.z
    box_marker.pose.orientation.w = points.pose.orientation.w
    box_marker.scale.x = 0.02
    box_marker.scale.y = 0.02
    box_marker.scale.z = 0.02
    if success:
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0

    else:
        box_marker.color.r = 1.0
        box_marker.color.g = 0.0
        box_marker.color.b = 0.0
    box_marker.color.a = 1.0
    pub.publish(box_marker)

def collect_data_samples(filepath, sensor_calibrator):
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub_markers =  rospy.Publisher("path_marker", Marker, queue_size = 100)

    # Creates the SimpleActionClient, passing the type of the action
    platform_server = rospy.get_param('~platform_server')
    platform_client = actionlib.SimpleActionClient(platform_server, PlatformGoalAction)

    print("Waiting for platform server - "+platform_server)
    platform_client.wait_for_server()
    print("Server Ready moving to calibration")

    #As we are moving the arm around without a calibrated sensor we use another refernece frame for controlling movement
    control_frame = rospy.get_param('~robot_control_frame')

    init_x = rospy.get_param('~init_x')
    init_y = rospy.get_param('~init_y')
    init_z = rospy.get_param('~init_z')
    init_roll  = rospy.get_param('~init_roll', 0)
    init_pitch = rospy.get_param('~init_pitch', 0)
    init_yaw   = rospy.get_param('~init_yaw', 90)

    planning_link = rospy.get_param('~planning_link')

    init_pose = utils.create_pose_stamped_msg(init_x, init_y, init_z, planning_link, rpy_deg=[init_roll,init_pitch,init_yaw])

    ns = rospy.get_namespace().strip("/")
    init_goal = utils.create_goal_msg(init_pose, PlatformGoalGoal.MOVE, control_frame)

    print("Sending init position", init_goal)
    platform_client.send_goal(init_goal)
    platform_client.wait_for_result()
    result = platform_client.get_state()
    if result != 3:
        print("failed to home")
        # return False

    print("Moved to initial position moving to data gathering phase")

    path_factory = PathFactory()

    #TODO pull this variable for path selection out...
    pathway = path_factory.create_path(2, planning_link)
    total = len(pathway['pathway'])
    print("Calibrating off "+str(total)+" locations")
    target_pose = pathway['target']
    mark_on_rviz(pub_markers, target_pose, planning_link, -1, success=True)

    for count, pose in enumerate(pathway['pathway']):
        pose_goal = utils.create_goal_msg(pose, PlatformGoalGoal.MOVE, control_frame)
        print("Moving too: "+str(count)+"/"+str(total))
        # print(pose_goal.target_pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()

        #handle result feedback - fail, success, etc
        result = platform_client.get_state()
        print("Result is ", result)

        if result == 3:# SUCCEEDED
            mark_on_rviz(pub_markers, pose, planning_link, count, success=True)

            rospy.sleep(2.0)#just to allow the shaking to stop
            file_name = filepath+str(count)
            sensor_calibrator.collect_samples(file_name, tf_buffer)
        else:
            print("Failed to reach pose")
            mark_on_rviz(pub_markers, pose, planning_link, count, success=False)

        if rospy.is_shutdown():
            return False

    print("Moving back to home position")
    platform_client.send_goal(init_goal)
    return True

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
    target_marker_id = rospy.get_param('~target_marker_id')
    sensor_calibrator = CalibratorFactory.create_calibrator(sensor, target_marker_id)

    if sensor_calibrator == None:
        print("Undefined Sensor Type: "+str(sensor))

    filepath = ""
    filepath = rospy.get_param('~file_path', filepath)
    image_list = []
    if filepath != "":
        print("Loading Data from file")
        sensor_calibrator.load_data(filepath)
    else:
        filepath = generate_filepath()
        if not collect_data_samples(filepath, sensor_calibrator):
            return

    print("Saving data too: "+filepath)

    sensor_calibrator.calibrate(filepath)

    print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
