#!/usr/bin/env python3
from os import path
import rospy
import roslib
import math 
import time
from datetime import datetime
from scipy.spatial.transform import Rotation as R

import numpy as np

import cv2
from cv_bridge import CvBridge

import tf
import tf2_ros
import tf2_geometry_msgs

import itertools


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from cares_msgs.srv import CalibrationService, ArucoDetect

from handeye_calibrator import HandeyeCalibrator 

import actionlib
from platform_msgs.msg import PlatformGoalAction, PlatformGoalGoal

import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory
from cares_lib_ros.data_sampler import StereoDataSampler

from os.path import expanduser
home = expanduser("~")

def loadData(filepath):
    image_list = []
    left_image, files  = utils.loadImages(filepath+"*left_rgb.png")
    right_image, files = utils.loadImages(filepath+"*right_rgb.png")
    transforms, files  = utils.load_transforms(filepath+"*transforms.yaml")
    assert len(left_image) == len(right_image) == len(transforms)
    for i in range(len(left_image)):
        file_name = filepath+str(i)+"_"
        image_list.append((left_image[i], right_image[i], transforms[i], file_name))
    return image_list




def move_arm(filepath):
    
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #As we are moving the arm around without a calibrated sensor we use the ee_link to move the arm
    ee_frame = rospy.get_param('~robot_effector_frame')

    #Global link to align data too
    #TODO extract this out via params
    world_link = "world"

    #Image sampler that will collect the sensor data
    image_sampler = StereoDataSampler()
 
    # Creates the SimpleActionClient, passing the type of the action
    platform_server = rospy.get_param('~platform_server', 'server')
    platform_client = actionlib.SimpleActionClient(platform_server, PlatformGoalAction)
    
    print("Waiting for platform server - "+platform_server)
    platform_client.wait_for_server()
    print("Server Ready moving to calibration")

    path_factory = PathFactory.scan_forward()
      
    init_goal = utils.create_goal_msg(path_factory.home, 0, ee_frame)
    # goal = utils.pose_to_rt(PathFactory.home)

    print("Goal transform:")
    print(utils.pose_to_rt(init_goal.target_pose))
    # print(goal)

    print("Sending init position", init_goal)
    platform_client.send_goal(init_goal)
    platform_client.wait_for_result()
    print("Moved to initial position moving to mapping phase")

        
    #TODO pull this variable for path selection out...
    total = 60
    
    image_list = []
    
    for pose in path_factory.calibrate:
        pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
        
        count = len(image_list)
        print(f"Capturing pose {count}/{total}")
        print(pose_goal.target_pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()
        
        #handle result feedback - fail, success, etc
        result = platform_client.get_state()
        print("Result is ", result)
        

        if  result == 3:# SUCCEEDED
            #########################LOGIC FOR TAKING IMAGES#################################
            rospy.sleep(0.5) #just to allow the shaking to stop
            image_sampler.sample_multiple_streams(rgb_image=True, depth_image=False, points=False, camera_info=False)

            sensor_timestamp = image_sampler.left_image_msg.header.stamp
            transform = tf_buffer.lookup_transform(world_link, ee_frame, sensor_timestamp, rospy.Duration(1.0))

            file_name = path.join(filepath, str(count))
            print(f"Saving {file_name}")

            image_list.append((image_sampler.left_image, image_sampler.right_image, transform, file_name))
            image_sampler.save(file_name)
            utils.save_transform(file_name+"_transforms.yaml", transform)
            ############################################################
        else:
            print("Failed to reach pose")

        if len(image_list) == total:
            break

        if rospy.is_shutdown():
            return
                            
    print("moving back to home position")
    platform_client.send_goal(init_goal)
    return image_list

def main():
    rospy.init_node('stereo_auto_calibration_node')
    np.set_printoptions(precision=4, suppress=True)

    filepath = rospy.get_param('~file_path', None)
    image_list = []
    if filepath is not None:
        image_list = loadData(filepath)
    else:
        now = datetime.now()
        now = now.strftime("%Y-%m-%d-%H-%M-%S")
        filepath = home +"/calibration_images/"+now+"/"
        import os
        if not os.path.exists(filepath):
            os.makedirs(filepath)
        image_list = move_arm(filepath)

    # print("Saving data too: "+filepath)

    # # Run calibration service
    # print("Running Stereo Calibration Service")
    # calibration_service_name = 'stereo_calibration'
    # calibration_service = rospy.ServiceProxy(calibration_service_name, CalibrationService)
    # result = calibration_service(filepath)
    # stereo_info = result.stereo_info
    
    # print("Stereo Information:")
    # print(stereo_info)
    
    # # Rectify the images before sending them to the marker detector service
    # images_left  = [img[0] for img in image_list]
    # images_right = [img[1] for img in image_list]
    # images_left_rectified, images_right_rectified = utils.rectify_images(images_left, images_right, stereo_info)

    # # Setup Charuco detection service
    # aruco_detect = rospy.ServiceProxy('aruco_detector', ArucoDetect)
    
    # # Setup hand eye calibrator
    # hand_eye_calibrator = HandeyeCalibrator()

    # bridge = CvBridge()
    # for i in range(len(images_left_rectified)):
    #     msg_left_rectified  = bridge.cv2_to_imgmsg(images_left_rectified[i], encoding="passthrough")
    #     msg_right_rectified = bridge.cv2_to_imgmsg(images_right_rectified[i], encoding="passthrough")

    #     # Get transform from charuco detection service
    #     aruco_transforms = aruco_detect(msg_left_rectified, msg_right_rectified, stereo_info)
    #     marker_id = 11
    #     if marker_id in aruco_transforms.ids:
    #         index = aruco_transforms.ids.index(marker_id)
    #         transform = aruco_transforms.transforms[index]
    #         sample = {'robot':image_list[i][2],'optical':transform}
    #         print(sample)
    #         hand_eye_calibrator.collect_samples(sample)
    #     else:
    #         print("Marker not detected")

    # # Compute the calibration
    # calibration = hand_eye_calibrator.compute_calibration()
    # print("Hand-eye calibration:")
    # print(calibration.to_dict())
    
    # if calibration is not None:
    #     calibration.to_file(filepath[:-1], "stereo_handeye")
    
    # print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
