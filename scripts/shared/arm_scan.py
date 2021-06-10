from os import path
import os
import re

import rospy
from datetime import datetime

import tf2_ros
import actionlib

from platform_msgs.msg import PlatformGoalAction, PlatformGoalGoal
from cares_lib_ros.path_factory import PathFactory
from cares_lib_ros.data_sampler import StereoDataSampler
from cares_lib_ros.container import transpose_list_dicts


from os.path import expanduser
from cares_lib_ros import utils, scan_files


def arm_scan(world_frame, ee_frame, file_path, image_sampler, total_images):

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    def capture_image(file_name):
        image_sampler.sample_multiple_streams(
            rgb_image=True, depth_image=False, points=False, camera_info=False)

        sensor_timestamp = image_sampler.left_image_msg.header.stamp
        transform = tf_buffer.lookup_transform(
            world_frame, ee_frame, sensor_timestamp, rospy.Duration(1.0))

        base_filename = path.join(file_path, file_name)
        rospy.loginfo(f"Saving image {base_filename}")

        image_sampler.save()
        utils.save_transform(f"{base_filename}_transforms.yaml", transform)
        return dict(left=image_sampler.left_image, right=image_sampler.right_image, 
          transform=transform, image_name=file_name)

    # Creates the SimpleActionClient, passing the type of the action
    platform_server = rospy.get_param('~platform_server', 'server')
    platform_client = actionlib.SimpleActionClient(
        platform_server, PlatformGoalAction)

    print("Waiting for platform server - "+platform_server)
    platform_client.wait_for_server()
    print("Server Ready moving to calibration")

    path_factory = PathFactory.scan_down()
    init_goal = utils.create_goal_msg(path_factory.home, 0, ee_frame)
    # goal = utils.pose_to_rt(PathFactory.home)

    rospy.loginfo("Goal transform:")
    rospy.loginfo(utils.pose_to_rt(init_goal.target_pose))

    rospy.loginfo("Sending init position", init_goal)
    platform_client.send_goal(init_goal)
    platform_client.wait_for_result()
    rospy.loginfo("Moved to initial position moving to mapping phase")

    image_list = []

    for pose in path_factory.calibrate:
        pose_goal = utils.create_goal_msg(pose, 0, ee_frame)

        count = len(image_list)
        print(f"Capturing pose {count}/{total_images}")
        print(pose_goal.target_pose)
        platform_client.send_goal(pose_goal)
        platform_client.wait_for_result()

        # handle result feedback - fail, success, etc
        result = platform_client.get_state()
        rospy.loginfo("Result is ", result)

        if result == 3:  # SUCCEEDED
            rospy.sleep(0.5)  # Allow the shaking to stops
            image_list.append(capture_image(file_name=str(count)))
        else:
            rospy.logwarn("Failed to reach pose")

        if len(image_list) == total_images:
            break

        if rospy.is_shutdown():
            return

    rospy.loginfo("Moving back to home position")
    platform_client.send_goal(init_goal)

    return transpose_list_dicts(image_list)


def dated_path(base_path="~/calibration_images"):
  now = datetime.now()
  now = now.strftime("%Y-%m-%d-%H-%M-%S")
  filepath = os.path.join(expanduser(base_path), str(now))
  if not os.path.exists(filepath):
      os.makedirs(filepath)

  return filepath



def acquire_images(file_path=None):

    ee_frame = rospy.get_param('~robot_effector_frame')
    world_frame = rospy.get_param('~world_frame', 'world')
    total_images = rospy.get_param('~num_images', 60)

    if file_path is not None:
        files = scan_files.find_stereo_scan(expanduser(file_path))
        rospy.loginfo(f"Found scan with {len(files['image_names'])} files, loading...")
        return files, scan_files.load_stereo_scan(files)
    else:
      file_path = dated_path()
      # Image sampler that will collect the sensor data
      image_sampler = StereoDataSampler()
      
      images = arm_scan(world_frame, ee_frame, 
        file_path, image_sampler, total_images) 
      files = scan_files.find_stereo_scan(file_path)
      
      return files, images