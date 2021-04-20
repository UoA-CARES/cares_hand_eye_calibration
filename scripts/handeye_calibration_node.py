#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import tf
import math 
import time

from cv_bridge import CvBridge

from maara_msgs.msg import KeyPoints, ArmCmd
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker

from move_group_python_interface import MoveGroupPythonInterface
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from math import pi

import numpy as np
import cv2

import matplotlib.pyplot as plt
import tf2_ros
import tf2_geometry_msgs

from calibration.handeye_calibrator import HandeyeCalibrator 

from datetime import datetime

from os.path import expanduser
home = expanduser("~")

def create_pose_msg(x, y, z, ox=0, oy=0, oz=0, ow=1):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    return pose

def searchTF(buf, from_tf, to_tf):
    return buf.lookup_transform(from_tf, to_tf, rospy.Time.now(), rospy.Duration(4))

def get_image_pose(ur5, source, buf):
    ee_link_pose = ur5.get_current_pose()
    aruco_link = rospy.get_param('~tracking_marker_frame')
    transform = searchTF(buf, source, aruco_link)
    pose_stamped = PoseStamped()
    pose_stamped.pose = Pose()
    estimated_point = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    return estimated_point

def get_rotation_flag():
    rotation_flag = rospy.get_param('~rotation_flag')
    if rotation_flag == 0:
        return "ryp"
    elif rotation_flag ==100:
        return "rpy"
    elif rotation_flag == 1:
        return "pry"
    elif rotation_flag == 10:
        return "yrp"
    elif rotation_flag == 11:
        return "pry2"
    elif rotation_flag == -100:
        return "rpy2"
    else:
        rospy.logerr("error not defined %s",rotation_flag)

def quarternion_to_rpy(quaternion):
    (r,p,y)= euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    if get_rotation_flag() == "rpy":
        return (r,p,y)
    elif get_rotation_flag() == "ryp":
        return (r,y,p)
    elif get_rotation_flag() == "pry":
        return (p,r,y)
    elif get_rotation_flag() == "pry2":
        return (r,y,p)
    elif get_rotation_flag() == "ypr":
        return (y,p,r)
    elif get_rotation_flag() == "rpy2":
        return (r,y,p)
    else:
        rospy.logerr("error not defined")

def rpy_to_quaternion(r,p,y):
    if get_rotation_flag() == "rpy":
        return quaternion_from_euler(r,p,y)
    elif get_rotation_flag() == "ryp":
        return quaternion_from_euler(r,y,p)
    elif get_rotation_flag() == "pry":
        return quaternion_from_euler(p,r,y)
    elif get_rotation_flag() == "ypr":
        return quaternion_from_euler(y,p,r)
    elif get_rotation_flag() == "pry2":
        return quaternion_from_euler(r,y,p)
    elif get_rotation_flag() == "rpy2":
        return quaternion_from_euler(r,y,p)
    else:
        rospy.logerr("error not defined")


def initial_rotation():
    if get_rotation_flag() == "ryp":
        return [0,0,0]
    elif get_rotation_flag() == "rpy":
        return [-math.pi/2,0,0]
    elif get_rotation_flag() == "pry":
        return [0,0,math.pi/2]
    elif get_rotation_flag() == "ypr":
        return [0,math.pi/2,0]
    elif get_rotation_flag() == "pry2":
        return [0,math.pi/2,math.pi/2] 
    elif get_rotation_flag() == "rpy2":
        return [-math.pi/2,0,0] 
    else:
        rospy.logerr("error not defined")

def main():
    # return
    rospy.init_node('move_group_python_interface', anonymous=True)
    time.sleep(5.0)
    #tf listener shit
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    ur5 = MoveGroupPythonInterface()
    # ur5.set_ee_link("d435")
    base_frame = rospy.get_param('~tracking_base_frame')
    print(base_frame)
    ur5.set_ee_link("ee_link")
    #position 0
    init_pose = create_pose_msg(0, 0.8 , 0.7)
    #ur5.add_exlusion_zone(0, 0.9, 0, 2.0, 0.02, 2.0, "wall") 
    print(ur5.get_current_pose())
    
    start_pose = init_pose
    start_pose.position.y -= 0.25
    rotation_rpy = initial_rotation()
    #for realsense facing down pitch is updown(positive down) yaw is roll
    q_orig = quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
    #q_offset = quaternion_from_euler(math.pi/6, 0, 0)
    #q = quaternion_multiply(q_offset, q)
    start_pose.orientation.x = q_orig[0]
    start_pose.orientation.y = q_orig[1]
    start_pose.orientation.z = q_orig[2]
    start_pose.orientation.w = q_orig[3]
    ur5.plan_pose_goal(start_pose)
    if ur5.go_to_pose_goal(start_pose):
        print(start_pose)
        print("moved")
    else:
        print("failed to reach initial pose")
    print("current pose", start_pose) 

    hand_eye_calibrator = HandeyeCalibrator()
    
    hand_eye_calibrator._wait_for_tf_init()
    time.sleep(2.0)
    if hand_eye_calibrator.take_sample():
        #print("Found")
        estimated_pose = get_image_pose(ur5, "world", tf_buffer)
        print("pose at",estimated_pose)
    else:
        print("NOT FOUND")
        return
    #create path plan around the estimated position0
    dx = 4
    dz = 3
    radius  = 0.7
    step_x = 0.12
    step_z = 0.1

    #print("MOVING")
    marker_pose = Pose()    
    marker_pose.position.x = estimated_pose.pose.position.x  
    marker_pose.position.y = estimated_pose.pose.position.y-radius
    marker_pose.position.z = estimated_pose.pose.position.z
    #rotation
    marker_pose.orientation.x = start_pose.orientation.x#estimated_pose.transform.rotation.x
    marker_pose.orientation.y = start_pose.orientation.y#estimated_pose.transform.rotation.y
    marker_pose.orientation.z = start_pose.orientation.z#estimated_pose.transform.rotation.z
    marker_pose.orientation.w = start_pose.orientation.w#estimated_pose.transform.rotation.w
    print("MOVING 2", marker_pose)
    ur5.go_to_pose_goal(marker_pose)
    
    #sample_size =  
    # dr = 4
    # # dp = 4
    # yaw = 
    for i in range(0,dx):
         for j in range(0,dz):
            for l in [-1,1]:
                for k in [-1,1]:
                    print(i, j , l)
                    #create path around estimated_pose
                    pose_goal = Pose()
                    pose_goal.position.x = estimated_pose.pose.position.x - radius*math.sin(l*step_x*i)
                    pose_goal.position.y = estimated_pose.pose.position.y - radius*math.cos(l*step_x*i)
                    pose_goal.position.z = estimated_pose.pose.position.z + radius*math.sin(k*step_z*j)
                    #(roll, pitch, yaw) = euler_from_quaternion([marker_pose.orientation.x,marker_pose.orientation.y,marker_pose.orientation.z,marker_pose.orientation.w])
                    #(roll, pitch, yaw) = quarternion_to_rpy(marker_pose.orientation)
                    #roll = roll + (step_z*j*k) #updown
                    #pitch = pitch + i*step_z*l #rotate
                    #yaw = yaw + step_x*i*l #sideways
                    roll_offset = rpy_to_quaternion(-step_z*j*k,0,0)
                    yaw_offset = rpy_to_quaternion(0,0,-step_x*i*l)

                    #q = rpy_to_quaternion(roll,pitch,yaw)
                    q = quaternion_multiply(yaw_offset, quaternion_multiply(roll_offset, q_orig))
                    (roll, pitch , yaw ) = euler_from_quaternion(q)
                    pose_goal.orientation.x =q[0]
                    pose_goal.orientation.y =q[1]
                    pose_goal.orientation.z =q[2]
                    pose_goal.orientation.w =q[3]
                    print("moving to ",(pose_goal.position.x,pose_goal.position.y,pose_goal.position.z),(roll,pitch,yaw))
                    if ur5.go_to_pose_goal(pose_goal):
                        time.sleep(4.0)
                        hand_eye_calibrator.take_sample()
                    else:
                        print("Failed to reach pose")

                    if rospy.is_shutdown():
                        return

    calibration = hand_eye_calibrator.compute_calibration()
    print(calibration)
    if calibration is not None:
        now = datetime.now()
        dt_string = now.strftime("%Y-%m-%d-%H-%M-%S")
        calibration.to_file(home+"/hand_eye_calibrations", "calibration-"+dt_string)

    print("Done calibration moving back to home position")
    ur5.go_to_pose_goal(start_pose)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
