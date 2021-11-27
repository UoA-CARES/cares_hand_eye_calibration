#!/usr/bin/env python3
import numpy as np
import rospy

import math
import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory, World, concat_lists


from camera_calibration.srv import Calibrate
from arm_utils import run_scan, get_client, wait_goal

def main():
  rospy.init_node('arm_calibration_node')
  client = get_client()
  ee_frame = rospy.get_param('~robot_effector_frame')

  def goto(pose):
    pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
    wait_goal(client, pose_goal)

  camera_topic = rospy.get_param('~camera_topic', "stereo_pair")
  calibrate = rospy.ServiceProxy(f'{camera_topic}/calibrate', Calibrate)

  home_position = np.array([0.2, 0.7, 0.8])
  calib_target = np.array([0.2, 1.0, 0.8])

  home_pose = PathFactory.look_at(home_position, home_position + World.forward, up=World.up)
  goto(home_pose)

  pathway = [
    PathFactory.calibration_scan(home_position, calib_target, size=(3, 3, 3), up=up)
      for up in [World.up, World.right, -World.up]]
  pathway = concat_lists(pathway)

  filename = run_scan(client, pathway, roll_tolerance=math.pi/2, ee_frame=ee_frame) 
  calibrate(scan_file=filename)

  goto(home_pose)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
