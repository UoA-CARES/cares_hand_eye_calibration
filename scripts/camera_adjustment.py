#! /usr/bin/env python

import rospy
import roslib
import math 
from tf.transformations import quaternion_from_euler, euler_from_quaternion 

from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped, Transform, TransformStamped

from move_group_python_interface import MoveGroupPythonInterface

from calibration.handeye_calibrator import HandeyeCalibrator 

import tf2_ros
import tf2_geometry_msgs
import time
import numpy as np


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

def initial_rotation():
    print("setting to ", get_rotation_flag())
    if get_rotation_flag() == "ryp":
        return [0,0,0]
    elif get_rotation_flag() == "rpy":
        return [-math.pi/2,0,0]
    elif get_rotation_flag() == "pry":
        return [0,0,math.pi/2]
    elif get_rotation_flag() == "ypr":
        return [0,math.pi/2,0]
    else:
        rospy.logerr("error not defined")

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
    elif get_rotation_flag() == "ypr":
        return (y,p,r)
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
    else:
        rospy.logerr("error not defined")

def searchTF(buf, from_tf, to_tf):
    # self.tfBuffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, rospy.Time(0), rospy.Duration(10))

    return buf.lookup_transform(from_tf, to_tf, rospy.Time.now(), rospy.Duration(4))

def get_image_pose(ur5, source, buf):
    ee_link_pose = ur5.get_current_pose()
    aruco_link = rospy.get_param('~tracking_marker_frame')
    transform = searchTF(buf, source, aruco_link)
    # pose_stamped = PoseStamped()
    # pose_stamped.pose = ee_link_pose
    # estimated_point = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    return transform

def averaging_poses(list_of_poses):
    x_sum=0.0
    y_sum=0.0
    z_sum=0.0
    qx_sum=0.0
    qy_sum=0.0
    qz_sum=0.0
    qw_sum=0.0
    n = len(list_of_poses)
    for p in list_of_poses:
        x_sum+=p.position.x
        y_sum+=p.position.y
        z_sum+=p.position.z
        qx_sum+=p.orientation.x
        qy_sum+=p.orientation.y
        qz_sum+=p.orientation.z
        qw_sum+=p.orientation.w

    return (x_sum/n, y_sum/n, z_sum/n, qx_sum/n, qy_sum/n, qz_sum/n, qw_sum/n)


def averaging_transforms(list_of_transforms):
    x_sum=0.0
    y_sum=0.0
    z_sum=0.0
    qx_sum=0.0
    qy_sum=0.0
    qz_sum=0.0
    qw_sum=0.0
    n = len(list_of_transforms)
    for t in list_of_transforms:
        x_sum+=t.translation.x
        y_sum+=t.translation.y
        z_sum+=t.translation.z
        qx_sum+=t.rotation.x
        qy_sum+=t.rotation.y
        qz_sum+=t.rotation.z
        qw_sum+=t.rotation.w

    return (x_sum/n, y_sum/n, z_sum/n, qx_sum/n, qy_sum/n, qz_sum/n, qw_sum/n)

def create_tf_transform_msg(translation, rotation, link_name, parent_link):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_link
    t.child_frame_id = link_name
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t

def create_tf_transform_msg_from_msg(transform_msg, link_name, parent_link):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_link
    t.child_frame_id = link_name
    t.transform = transform_msg
    return t


def main():
    rospy.init_node('move_group_python_interface', anonymous=True)
    time.sleep(3.0)
    #tf listener shit
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    ur5 = MoveGroupPythonInterface()
    # ur5.set_ee_link("d435")
    base_frame = rospy.get_param('~tracking_base_frame')
    print(base_frame)
    ur5.set_ee_link(base_frame)
    # ur5.set_ee_link("zivid_calibration_link")

    ###############################################
    tf_A= rospy.get_param('~tf_A')
    tf_B= rospy.get_param('~tf_B')
    marker_tf_sensor_1= rospy.get_param('~marker_tf_sensor_1')
    marker_tf_sensor_2= rospy.get_param('~marker_tf_sensor_2')
    br = tf2_ros.TransformBroadcaster()

    ##################################################


    
    init_pose = create_pose_msg(-0.1, 0.9, 0.8)
    start_pose = init_pose
    start_pose.position.y -= 0.25
    rotation_rpy = initial_rotation()
    q = quaternion_from_euler(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])
    start_pose.orientation.x = q[0]
    start_pose.orientation.y = q[1]
    start_pose.orientation.z = q[2]
    start_pose.orientation.w = q[3]

    if ur5.go_to_pose_goal(start_pose):
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
        #print(estimated_pose)
    else:
        print("NOT FOUND")
        return
   
    #create path plan around the estimated position0
    dx = 3
    dz = 3
    radius  = 0.5
    step_x = 0.12
    step_z = 0.1

    #print("MOVING")
    marker_pose = Pose()    
    marker_pose.position.x = estimated_pose.transform.translation.x  
    marker_pose.position.y = estimated_pose.transform.translation.y-radius
    marker_pose.position.z = estimated_pose.transform.translation.z
    #rotation
    marker_pose.orientation.x = start_pose.orientation.x#estimated_pose.transform.rotation.x
    marker_pose.orientation.y = start_pose.orientation.y#estimated_pose.transform.rotation.y
    marker_pose.orientation.z = start_pose.orientation.z#estimated_pose.transform.rotation.z
    marker_pose.orientation.w = start_pose.orientation.w#estimated_pose.transform.rotation.w
    #print("MOVING 2", marker_pose)
    #ur5.go_to_pose_goal(marker_pose)
    

    debug_transform = Transform()
    list_of_poses = []
    list_of_transform = []
    realsense_to_zivid = tf_buffer.lookup_transform("camera_link", "zivid_optical_frame" , rospy.Time(0), rospy.Duration(2))
    zivid_to_realsense = tf_buffer.lookup_transform("zivid_optical_frame", "camera_link" , rospy.Time(0), rospy.Duration(2))
    print(zivid_to_realsense)
    print(realsense_to_zivid) 
    
    for i in range(0,dx):
         for j in range(0,dz):
            for l in [1]:
                for k in [1]:
                    try:
                        print(i, j , l)
                        #create path around estimated_pose
                        pose_goal = Pose()
                        pose_goal.position.x = estimated_pose.transform.translation.x + radius*math.sin(l*step_x*i)
                        pose_goal.position.y = estimated_pose.transform.translation.y - radius*math.cos(l*step_x*i)
                        pose_goal.position.z = estimated_pose.transform.translation.z + radius*math.sin(k*step_z*j)
                        #(roll, pitch, yaw) = euler_from_quaternion([marker_pose.orientation.x,marker_pose.orientation.y,marker_pose.orientation.z,marker_pose.orientation.w])
                        (roll, pitch, yaw) = quarternion_to_rpy(marker_pose.orientation)
                        roll = roll - (step_z*j*k) #updown
                        #pitch = pitch + i*step_z*l #rotate
                        yaw = yaw + step_x*i*l #sideways
                        #q = quaternion_from_euler(roll, pitch ,yaw)
                        q = rpy_to_quaternion(roll,pitch,yaw)
                        pose_goal.orientation.x =q[0]
                        pose_goal.orientation.y =q[1]
                        pose_goal.orientation.z =q[2]
                        pose_goal.orientation.w =q[3]
                        print("moving to ",(pose_goal.position,pose_goal.orientation),(roll,pitch,yaw))
                        ur5.set_ee_link("zivid_optical_frame")
                        if ur5.go_to_pose_goal(pose_goal):
                            #RECORD ZIVID at P
                            time.sleep(5.0)
                            transform_zivid_marker = tf_buffer.lookup_transform("world", marker_tf_sensor_1 , rospy.Time(0), rospy.Duration(2))
                            pose_zivid_marker = tf2_geometry_msgs.do_transform_pose(PoseStamped(), transform_zivid_marker)
                            ur5.set_ee_link("camera_link")
                            pose_goal_stamped = PoseStamped()
                            pose_goal_stamped.pose = pose_goal
                            pose_goal_realsense = tf2_geometry_msgs.do_transform_pose(pose_goal_stamped, zivid_to_realsense)
                            if ur5.go_to_pose_goal(pose_goal_realsense.pose):
                                time.sleep(5.0)
                                transform_realsense_marker = tf_buffer.lookup_transform("world", marker_tf_sensor_2 , rospy.Time(0), rospy.Duration(2))
                                pose_realsense_marker = tf2_geometry_msgs.do_transform_pose(PoseStamped(), transform_realsense_marker)
                                print(transform_zivid_marker)
                                print(transform_realsense_marker)
                                #############################################################################  
                                p = Pose()
                                p.position.x = pose_zivid.pose.position.x - pose_realsense.pose.position.x + pose_zivid_marker.pose.position.x - pose_realsense_marker.pose.position.x
                                p.position.y = pose_zivid.pose.position.y - pose_realsense.pose.position.y + pose_zivid_marker.pose.position.y - pose_realsense_marker.pose.position.y
                                p.position.z = pose_zivid.pose.position.z - pose_realsense.pose.position.z + pose_zivid_marker.pose.position.z - pose_realsense_marker.pose.position.z
                                p.orientation.x = pose_zivid.pose.orientation.x - pose_realsense.pose.orientation.x + pose_zivid_marker.pose.orientation.x - pose_realsense_marker.pose.orientation.x
                                p.orientation.y = pose_zivid.pose.orientation.y - pose_realsense.pose.orientation.y + pose_zivid_marker.pose.orientation.y - pose_realsense_marker.pose.orientation.y
                                p.orientation.z = pose_zivid.pose.orientation.z - pose_realsense.pose.orientation.z + pose_zivid_marker.pose.orientation.z - pose_realsense_marker.pose.orientation.z
                                p.orientation.w = pose_zivid.pose.orientation.w - pose_realsense.pose.orientation.w + pose_zivid_marker.pose.orientation.w - pose_realsense_marker.pose.orientation.w
                                print("got transform!")
                                list_of_poses.append(p)
                            else:
                                print("Failed to reach pose for realsense")
                                continue
                            # Determine Pose Pz
                            # Move zivid to Pz
                                # Get pose of marker_0 realtive to Zivid
                            # Move RealSense to Pz + ---->Transform<----
                                # Get pose of realsense marker_0 relative to Realsense
                            # Calculate the difference between marker poses
                        else:
                            print("Failed to reach pose for zivid")

                    except tf2_ros.ExtrapolationException as extra_err:
                        print(extra_err)

                    if rospy.is_shutdown():
                        return

    #########################FINAL CALCULATION SHOULD BE HERE################################################
    #########################################################################    
    print("DONE")    
    #world to camera_link    
    calibration = averaging_poses(list_of_poses)    
    #calibration = (0.07765993478763315, 0.011317778913710622, 0.06120923691311283, 0.3976482583053898, 0.028438580039113773, -0.07690527436364158, 0.9138668830216832)
    print(calibration)
    rate = rospy.Rate(1.0)
    adjusted_pose = Pose()
    adjusted_pose.position.x = start_pose.position.x + calibration[0]
    adjusted_pose.position.y = start_pose.position.y + calibration[1]
    adjusted_pose.position.z = start_pose.position.z + calibration[2]
    adjusted_pose.orientation.x = start_pose.orientation.x
    adjusted_pose.orientation.y = start_pose.orientation.y
    adjusted_pose.orientation.z = start_pose.orientation.z
    adjusted_pose.orientation.w = start_pose.orientation.w

    while not rospy.is_shutdown():
        ur5.go_to_pose_goal(start_pose)
        time.sleep(5)
        ur5.go_to_pose_goal(adjusted_pose)
        time.sleep(5)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass