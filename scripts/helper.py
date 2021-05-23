#!/usr/bin/env python3
import rospy
import roslib
import tf
import math 
import time
import yaml

from cv_bridge import CvBridge

from glob import glob

from cv_bridge import CvBridge

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped, TransformStamped

from sensor_msgs.msg import Image, CameraInfo

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from math import pi

import numpy as np
import cv2

import tf2_ros
import tf2_geometry_msgs

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

# TODO FUTURE HENRY FIX THE TYPO'S on the FLAGS!
# TODO FUTURE FUTURE HENRY - remove the need for the flags or at least make intuiative

def get_rotation_flag():
    return rospy.get_param('~rotation_flag')
    # if rotation_flag == 0:
    #     return "ryp"
    # elif rotation_flag ==100:
    #     return "rpy"
    # elif rotation_flag == 1:
    #     return "pry"
    # elif rotation_flag == 10:
    #     return "yrp"
    # elif rotation_flag == 11:
    #     return "ryp2"
    # elif rotation_flag == -100:
    #     return "rpy2"
    # else:
    #     rospy.logerr("error not defined %s",rotation_flag)

def quarternion_to_rpy(quaternion):
    (r,p,y)= euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    if get_rotation_flag() == "rpy":
        return (r,p,y)
    elif get_rotation_flag() == "ryp":
        return (r,y,p)
    elif get_rotation_flag() == "pry":
        return (p,r,y)
    elif get_rotation_flag() == "ryp2":
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
    elif get_rotation_flag() == "ryp2":
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
    elif get_rotation_flag() == "ryp2":
        return [0,math.pi/2,math.pi/2] 
    elif get_rotation_flag() == "rpy2":
        return [-math.pi,0,0] 
    else:
        rospy.logerr("error not defined")

def save_transform(filename, transform):
    transform_dic = {}
    translation_dic = {}
    rotation_dic = {}
    translation_dic["x"] = transform.transform.translation.x
    translation_dic["y"] = transform.transform.translation.y
    translation_dic["z"] = transform.transform.translation.z
    rotation_dic["x"] = transform.transform.rotation.x
    rotation_dic["y"] = transform.transform.rotation.y
    rotation_dic["z"] = transform.transform.rotation.z
    rotation_dic["w"] = transform.transform.rotation.w
    transform_dic["translation"] = translation_dic
    transform_dic["rotation"] = rotation_dic
    
    with open(filename, "w") as file:
        documents = yaml.dump(transform_dic, file)

def read_transforms(f):
    with open(f) as file:
        t_map = yaml.load(file, Loader=yaml.Loader)
        t = TransformStamped()
        t.transform.translation.x = t_map["translation"]["x"]
        t.transform.translation.y = t_map["translation"]["y"]
        t.transform.translation.z = t_map["translation"]["z"]
        t.transform.rotation.x = t_map["rotation"]["x"]
        t.transform.rotation.y = t_map["rotation"]["y"]
        t.transform.rotation.z = t_map["rotation"]["z"]
        t.transform.rotation.w = t_map["rotation"]["w"]
        return t

def load_transforms(path):
    files = glob(path)
    files.sort()
    print("Found {} transforms at {}".format(len(tuple(files)), path))
    
    transforms = []
    for i, f in enumerate(files):
        try:
            img = read_transforms(f)
            transforms.append(img)
        except KeyError as e:
            print("error at ", i)
            print(e)
            raise

    return transforms, files

def loadImage(path):
    imgs = []
    files = glob(path)
    files.sort()
    print("Found {} images at {}".format(len(tuple(files)), path))

    for i, f in enumerate(files):
        img = cv2.imread(f)
        imgs.append(img)
    return imgs, files

def rectify_remap(images_left, images_right, stereo_info):
    w = stereo_info.left_info.width
    h = stereo_info.left_info.height

    K1 = np.reshape(stereo_info.left_info.K, (3,3))
    R1 = np.reshape(stereo_info.left_info.R, (3,3))
    P1 = np.reshape(stereo_info.left_info.P, (3,4))
    D1 = np.array(stereo_info.left_info.D)
    
    K2 = np.reshape(stereo_info.right_info.K, (3,3))
    R2 = np.reshape(stereo_info.right_info.R, (3,3))
    P2 = np.reshape(stereo_info.right_info.P, (3,4))
    D2 = np.array(stereo_info.right_info.D)
    
    # Q = np.reshape(stereo_info.Q, (4,4))
    # R = np.reshape(stereo_info.R_left_right,(3,3))
    # T = np.array(stereo_info.T_left_right)
    
    left_map_x, left_map_y   = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w,h), cv2.CV_32FC1)
    right_map_x, right_map_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w,h), cv2.CV_32FC1)
    images_left_rectified = []
    images_right_rectified = []
    for i in range(len(images_left)):
        left_rectified  = cv2.remap(images_left[i], left_map_x, left_map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)    
        right_rectified = cv2.remap(images_right[i], right_map_x, right_map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        images_left_rectified.append(left_rectified)
        images_right_rectified.append(right_rectified) 
    return images_left_rectified, images_right_rectified