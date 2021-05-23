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

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from cares_msgs.srv import CalibrationService, ArucoDetect

from move_group_python_interface import MoveGroupPythonInterface
from handeye_calibrator import HandeyeCalibrator 
from data_sampler import StereoDataSampler
import helper as helper

from os.path import expanduser
home = expanduser("~")

def loadData(filepath):
    image_list = []
    left_image, files  = helper.loadImage(filepath+"*left_rgb.png")
    right_image, files = helper.loadImage(filepath+"*right_rgb.png")
    transforms, files  = helper.load_transforms(filepath+"*transforms.yaml")
    assert len(left_image) == len(right_image) ==  len(transforms)
    for i in range(len(left_image)):
        file_name = filepath+str(i)+"_"
        image_list.append((left_image[i], right_image[i], transforms[i], file_name))
    return image_list

def move_arm(filepath):
    image_list = []
    
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    ur5 = MoveGroupPythonInterface()
    # ee_frame = rospy.get_param('~robot_effector_frame')
    ee_frame = "stereo_pair/left"
    ur5.set_ee_link(ee_frame)

    print(ur5.get_current_pose())

    #DEAR FUTURE HENRY!

    #RIGHT HAND RULE control
    #CORRECTION SHOULD BE THE SAME FOR ALL OF THEM


    image_sampler = StereoDataSampler()
    d_roll  = image_sampler.d_roll
    d_pitch = image_sampler.d_pitch
    d_yaw   = image_sampler.d_yaw

    init_pose    = helper.create_pose_msg(0, 0.75, 0.8)
    # rotation_rpy = helper.initial_rotation()
    q_orig = quaternion_from_euler(d_roll, d_pitch, d_yaw)
    roll_transform = quaternion_from_euler(0, 0, 0)
    q_orig = quaternion_multiply(q_orig, roll_transform)
    # q_orig = helper.quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
    init_pose.orientation.x = q_orig[0]
    init_pose.orientation.y = q_orig[1]
    init_pose.orientation.z = q_orig[2]
    init_pose.orientation.w = q_orig[3]

    world_link = "world"

    if ur5.go_to_pose_goal(init_pose):
        print("Moved to initial pose")
    else:
        print("failed to reach initial pose")
    print("current pose", init_pose) 

    # init_pose  = helper.create_pose_msg(0, 1.0, 0.8)
    # world_link = "world"

    # start_poses = []
    # increment = [[0,0],[0,0.07], [0,-0.07], [0.05, 0], [-0.05,0]]

    # for i in range(len(increment)):
    #     start_pose = Pose()
    #     start_pose.position.x = init_pose.position.x+increment[i][0]
    #     start_pose.position.y = init_pose.position.y+increment[i][1]
    #     start_pose.position.z = init_pose.position.z
    #     start_pose.position.y -= 0.25
    #     rotation_rpy = helper.initial_rotation()
    #     q_orig = helper.quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
    #     start_pose.orientation.x = q_orig[0]
    #     start_pose.orientation.y = q_orig[1]
    #     start_pose.orientation.z = q_orig[2]
    #     start_pose.orientation.w = q_orig[3]
    #     start_poses.append(start_pose)

    # if ur5.go_to_pose_goal(start_poses[0]):
    #     print("Moved to initial pose")
    # else:
    #     print("failed to reach initial pose")
    # print("current pose", start_poses[0]) 
    
    # #create path plan around the estimated position0
    # dx = 2
    # dz = 3
    # radius  = 0.7
    # step_x = 0.2
    # step_z = 0.2
    
    # count = 0
    # for m in range(0, len(start_poses)):
    #     for i in range(0,dx):
    #          for j in range(0,dz):
    #             for l in [-1,1]:
    #                 for k in [-1,1]:
    #                     #create path around estimated_pose
    #                     pose_goal = Pose()
    #                     pose_goal.position.x = start_poses[m].position.x - radius*math.sin(l*step_x*i)
    #                     pose_goal.position.z = radius*math.cos(l*step_x*i)
    #                     pose_goal.position.y = start_poses[m].position.y + radius*math.sin(k*step_z*j)
                        
    #                     roll_offset = helper.rpy_to_quaternion(-step_z*j*k,0,0)
    #                     yaw_offset  = helper.rpy_to_quaternion(0,0,-step_x*i*l)

    #                     q = quaternion_multiply(yaw_offset, quaternion_multiply(roll_offset, q_orig))
    #                     (roll, pitch , yaw ) = euler_from_quaternion(q)
    #                     pose_goal.orientation.x =q[0]
    #                     pose_goal.orientation.y =q[1]
    #                     pose_goal.orientation.z =q[2]
    #                     pose_goal.orientation.w =q[3]

    #                     print("Moving to ",(pose_goal.position.x,pose_goal.position.y,pose_goal.position.z),(roll,pitch,yaw))
    #                     if ur5.go_to_pose_goal(pose_goal):
    #                         #########################LOGIC FOR TAKING IMAGES#################################
    #                         image_sampler.sample_multiple_streams(rgb_image=True, depth_image=False, points=False, camera_info=False)
                            
    #                         sensor_timestamp = image_sampler.left_image_msg.header.stamp
    #                         transform = tf_buffer.lookup_transform(world_link, ee_frame, sensor_timestamp, rospy.Duration(1.0))
    #                         file_name = filepath+str(count)
                            
    #                         image_list.append((image_sampler.left_image, image_sampler.right_image, transform, file_name))
    #                         image_sampler.save(file_name)
    #                         helper.save_transform(file_name+"_transforms.yaml", transform)
    #                         count +=1
    #                         ############################################################
    #                     else:
    #                         print("Failed to reach pose")

    #                     if rospy.is_shutdown():
    #                         return
                            
    # print("moving back to home position")
    # ur5.go_to_pose_goal(start_poses[0])
    return image_list

def main():
    rospy.init_node('stereo_auto_calibration_node')

    from_file = False;
    filepath = ""
    image_list = []
    if from_file:
        filepath = "/home/anyone/calibration_images/2021-05-18-10-18-05/"
        image_list = loadData(filepath)
    else:
        # now = datetime.now()
        # now = now.strftime("%Y-%m-%d-%H-%M-%S")
        # filepath = home +"/calibration_images/"+now+"/"
        # import os
        # if not os.path.exists(filepath):
        #     os.makedirs(filepath)
        image_list = move_arm(filepath)
    return
    print("Saving data too: "+filepath)

    # Run calibration service
    print("Running Stereo Calibration Service")
    calibration_service_name = 'stereo_calibration'
    calibration_service = rospy.ServiceProxy(calibration_service_name, CalibrationService)
    result = calibration_service(filepath)
    stereo_info = result.stereo_info
    
    print("Stereo Information:")
    print(stereo_info)
    
    # Rectify the images before sending them to the marker detector service
    images_left  = [img[0] for img in image_list]
    images_right = [img[1] for img in image_list]
    images_left_rectified, images_right_rectified = helper.rectify_remap(images_left, images_right, stereo_info)

    # Setup Charuco detection service
    aruco_detect = rospy.ServiceProxy('aruco_detector', ArucoDetect)
    
    # Setup hand eye calibrator
    hand_eye_calibrator = HandeyeCalibrator()

    bridge = CvBridge()
    for i in range(len(images_left_rectified)):
        msg_left_rectified  = bridge.cv2_to_imgmsg(images_left_rectified[i], encoding="passthrough")
        msg_right_rectified = bridge.cv2_to_imgmsg(images_right_rectified[i], encoding="passthrough")

        # Get transform from charuco detection service
        aruco_transforms = aruco_detect(msg_left_rectified, msg_right_rectified, stereo_info)
        marker_id = 11
        if marker_id in aruco_transforms.ids:
            index = aruco_transforms.ids.index(marker_id)
            transform = aruco_transforms.transforms[index]
            sample = {'robot':image_list[i][2],'optical':transform}
            print(sample)
            hand_eye_calibrator.collect_samples(sample)
        else:
            print("Marker not detected")

    # Compute the calibration
    calibration = hand_eye_calibrator.compute_calibration()
    print("Hand-eye calibration:")
    print(calibration.to_dict())
    
    if calibration is not None:
        calibration.to_file(filepath[:-1], "stereo_handeye")
    
    print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
