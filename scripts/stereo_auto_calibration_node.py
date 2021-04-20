#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import tf
import math 
import time
import yaml

from cv_bridge import CvBridge

from glob import glob

from cv_bridge import CvBridge

from maara_msgs.msg import KeyPoints, ArmCmd
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped, TransformStamped

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
from std_srvs.srv import Empty, EmptyResponse
from data_sampler import DataSamplerStereoDepth

from charuco_calibrate import charuco_main, save_calibration, rectify_remap
from cares_msgs.srv import CalibrationService, CharucoDetect

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
        return [-math.pi,0,0] 
    else:
        rospy.logerr("error not defined")

#def detect(left, right, Q):
    #for i in left:


def loadImage(path):
    """
    Given a path, it reads all images. This uses glob to 
    grab file names and excepts wild cards *
    Ex. getImages('./images/*.jpg')
    """
    imgs = []
    files = glob(path)
    files.sort()  # put in order

    print("Found {} images at {}".format(len(tuple(files)), path))
    # print('-'*40)

    for i, f in enumerate(files):
        img = cv2.imread(f)
        imgs.append(img)
    return imgs, files



def load_transforms(path):
    files = glob(path)
    files.sort()
    print("Found {} transforms at {}".format(len(tuple(files)), path))
    # print('-'*40)
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


def read_transforms(f):
    with open(f) as file:
        t_map = yaml.load(file)
        t = TransformStamped()
        t.transform.translation.x = t_map["translation"]["x"]
        t.transform.translation.y = t_map["translation"]["y"]
        t.transform.translation.z = t_map["translation"]["z"]
        t.transform.rotation.x = t_map["rotation"]["x"]
        t.transform.rotation.y = t_map["rotation"]["y"]
        t.transform.rotation.z = t_map["rotation"]["z"]
        t.transform.rotation.w = t_map["rotation"]["w"]
        return t



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
    print(transform_dic)
    with open(filename, "w") as file:
        documents = yaml.dump(transform_dic, file)


def loadData(path):
    image_list = []
    left_image, files = loadImage(path+"*left_rgb.png")
    right_image, files = loadImage(path+"*right_rgb.png")
    transforms, files = load_transforms(path+"*transforms.yaml")
    assert len(left_image) == len(right_image) ==  len(transforms)
    for i in range(len(left_image)):
        file_name = path+str(i)+"_"
        image_list.append((left_image[i], right_image[i], transforms[i], file_name))
    return image_list

def move_arm():
    #tf listener shit
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    ur5 = MoveGroupPythonInterface()
    # ur5.set_ee_link("d435")
    base_frame = rospy.get_param('~tracking_base_frame')
    ee_frame = rospy.get_param('~robot_effector_frame')
    print(ee_frame)
    ur5.set_ee_link(ee_frame)
    # ur5.set_ee_link("zivid_calibration_link")

    init_pose = create_pose_msg(0, 1.0, 0.8)
    #ur5.add_exlusion_zone(0, 0.9, 0, 2.0, 0.02, 2.0, "wall") 
    world_link = "world"
    #print(ur5.get_current_pose())
    start_poses = []
    increment = [[0,0],[0,0.05], [0,-0.05], [0.05, 0], [-0.05,0]]
    for i in range(len(increment)):
        start_pose = Pose()
        start_pose.position.x = init_pose.position.x+increment[i][0]
        start_pose.position.y = init_pose.position.y+increment[i][1]
        start_pose.position.z = init_pose.position.z
        start_pose.position.y -= 0.25
        rotation_rpy = initial_rotation()
        q_orig = quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
        start_pose.orientation.x = q_orig[0]
        start_pose.orientation.y = q_orig[1]
        start_pose.orientation.z = q_orig[2]
        start_pose.orientation.w = q_orig[3]
        start_poses.append(start_pose)


    if ur5.go_to_pose_goal(start_poses[0]):
        print("moved")
    else:
        print("failed to reach initial pose")
    print("current pose", start_poses[0]) 
    now = datetime.now()
    now = now.strftime("%Y-%m-%d-%H-%M-%S")
    filepath = home +"/scans/"+now+"/"
    import os
    if not os.path.exists(filepath):
        os.makedirs(filepath)
    print("Saving data too: "+filepath)
    


    #create path plan around the estimated position0
    dx = 2
    dz = 2
    radius  = 0.6
    step_x = 0.2
    step_z = 0.2
    print("waiting for service")
    time.sleep(4.0)
    image_sampler = DataSamplerStereoDepth(save_rgb=True)
    image_list = []
    count = 0
    for m in range(0, len(start_poses)):
        for i in range(0,dx):
             for j in range(0,dz):
                for l in [-1,1]:
                    for k in [-1,1]:
                        print(i, j , l)
                        #create path around estimated_pose
                        pose_goal = Pose()
                        pose_goal.position.x = start_poses[m].position.x - radius*math.sin(l*step_x*i)
                        pose_goal.position.z = radius*math.cos(l*step_x*i)
                        pose_goal.position.y = start_poses[m].position.y + radius*math.sin(k*step_z*j)
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
                            #########################LOGIC FOR TAKING IMAGES#################################
                            image_sampler.sample_multiple_streams(rgb_image=True,\
                                                        camera_info=False,\
                                                        display=False)
                            sensor_timestamp = image_sampler.left_image_msg.header.stamp
                            transform = tf_buffer.lookup_transform(world_link, ee_frame, sensor_timestamp, rospy.Duration(1.0))
                            file_name = filepath+str(count)+"_"
                            image_list.append((image_sampler.left_image, image_sampler.right_image, transform, file_name))
                            image_sampler.save(file_name, False)
                            save_transform(file_name+"transforms.yaml", transform)
                            count +=1
                            ############################################################
                        else:
                            print("Failed to reach pose")

                        if rospy.is_shutdown():
                            return
    print("moving back to home position")
    ur5.go_to_pose_goal(start_poses[0])
    return image_list, filepath
    

RUNMODE = 0

PATH = "/home/anyone/scans/"

def save_calibration(K1,K2,imageSize,dist1, dist2, R, T, P1, R1, P2, R2, Q, imageSet ,root_path):
    calib = {}    
    #cam1    
    camera1_calibration ={}
    camera1_calibration["image_size"]=imageSize
    camera1_calibration["K"] = K1
    camera1_calibration["dist"] = dist1
    #camera 2
    camera2_calibration ={}
    camera2_calibration["image_size"]=imageSize
    camera2_calibration["K"] = K2
    camera2_calibration["dist"] = dist2
    #extrinsics
    camera_extrinsics = {}
    camera_extrinsics["R"] = np.linalg.inv(R)
    camera_extrinsics["T"] = T/-1000.0
    camera_extrinsics["parent"] ="cam1"
    camera_extrinsics["Q"]=Q
    #total
    calib={}     
    calib["cameras"] = {"cam1":camera1_calibration, "cam2":camera2_calibration}
    calib["extrinsics"]  = {"cam2":camera_extrinsics}
    calib["stereo_pairs"] = {"stereo1":["cam1", "cam2"]}
    calib["image_sets"] = {"rgb":imageSet}
    import json
    file_name = root_path+"stereo_pair_uoa.json"
    with open(file_name,"w") as fp:
        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                return json.JSONEncoder.default(self, obj)
        json.dump(calib, fp, cls=NumpyEncoder)
        print("saved ",file_name)

def main():
    # return
    rospy.init_node('stereo_auto_calibration_node')
    time.sleep(5.0)
    #if arm is active
    #image_list, filepath = move_arm()
    filepath = "/home/anyone/scans/2021-04-15-13-54-49/"
    image_list = loadData(filepath) 

    #####################################################
    #should have image lists
    #do calibration
    #M1, M2, d1, d2, w, h, R, T, Q = charuco_main(left_image_list, right_image_list)
    left_path = []
    right_path = []
    for c in range(len(image_list)):
        left_path.append(filepath+str(c)+"_left_rgb.png")
        right_path.append(filepath+str(c)+"_right_rgb.png")
    calibration_service = rospy.ServiceProxy('stereo_calibration', CalibrationService)
    #imageSet.append(left_path)
    #imageSet.append(right_path)
    #save_calibration(M1,M2,(w,h),d1,d2,R,np.transpose(T)[0],imageSet,ROOT_PATH)
    res = calibration_service(filepath)
    stereoinfo = res.stereo_info
    #####################################################
    ##do hand eye
    #rectify the images first
    imgsL = [img[0] for img in image_list]
    imgsR = [img[1] for img in image_list]

    M1 = np.reshape(stereoinfo.left_info.K,(3,3))
    M2 = np.reshape(stereoinfo.right_info.K,(3,3))
    d1 = stereoinfo.left_info.D
    d2 = stereoinfo.right_info.D
    w = stereoinfo.left_info.width
    h = stereoinfo.left_info.height
    R = np.reshape(stereoinfo.R_left_right,(3,3))
    T = np.array(stereoinfo.T_left_right)

    recL, recR, P1, P2, R1, R2, Q = rectify_remap(imgsL, imgsR, M1, d1, M2, d2, w, h, R, T)
    imageSet = [left_path, right_path]
    save_calibration(M1,M2,(w,h),d1, d2, R, T, P1, R1, P2, R2, Q, imageSet ,filepath)
    print(stereoinfo)
    hand_eye_calibrator = HandeyeCalibrator()
    print("starting proxy")
    charuco_detect = rospy.ServiceProxy('charuco_detector', CharucoDetect)
    print("proxy created")
    bridge = CvBridge()
    for i in range(len(recL)):
        #get pose
        recL_msg = bridge.cv2_to_imgmsg(recL[i], encoding="passthrough")
        recR_msg = bridge.cv2_to_imgmsg(recR[i], encoding="passthrough")
        transform_from_camera_to_marker = charuco_detect(recL_msg, recR_msg, stereoinfo)
        if transform_from_camera_to_marker.transform.transform.translation.x == 0 and transform_from_camera_to_marker.transform.transform.translation.y ==0 and transform_from_camera_to_marker.transform.transform.translation.z == 0:
            continue
        sample = {'robot':image_list[i][2],'optical':transform_from_camera_to_marker.transform}
        print(sample)
        hand_eye_calibrator.collect_samples(sample)
    calibration = hand_eye_calibrator.compute_calibration()
    print(calibration)
    if calibration is not None:
        now = datetime.now()
        dt_string = now.strftime("%Y-%m-%d-%H-%M-%S")
        calibration.to_file(filepath[:-1], "calibration-"+dt_string)
    #####################################################
    print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
