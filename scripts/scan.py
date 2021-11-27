#!/usr/bin/env python3
import numpy as np
import rospy


import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory, roll_variations, World


import cares_lib_ros.utils as utils
from arm_utils import run_scan, wait_goal, get_client

import math



def main():
  rospy.init_node('arm_scan_node')
  
  distance = 0.8
  height = 0.9

  arc_params = dict(
    segments=8, 
    arc_range=(-math.pi/4, math.pi/4),
    radius=0.3,
    axes=World
  )



  client = get_client()
  branch_position = np.array([0.2, distance, height])
  home_position = branch_position - World.forward * 0.2


  ee_frame = rospy.get_param('~robot_effector_frame')

  def goto(pose):
    pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
    wait_goal(client, pose_goal)


  def cartesian_pathway(pathway):
      goal = utils.create_multi_goal_msg(pathway, 0, ee_frame, cartesian_path=True)
      wait_goal(client, goal)



  pathway1 = PathFactory.arc_scan(home_position, axis_range=(-0.25, 0.25), stops=8, **arc_params)
  pathway2 = PathFactory.single_arc(origin=home_position, **arc_params)


  # status = run_scan(client, pathway2, "scan") 

  cartesian_pathway(pathway2)


  # cartesian_pathway(pathway[:-1] + list(reversed(pathway)))




  # client.wait_for_result()


  # pathway = PathFactory.arc_scan(branch_position)
  # status = run_scan(client, pathway, "scan") 




  # Return to home position
  home = PathFactory.look_at(home_position, branch_position, up=World.up)
  goto(home)



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
