#!/usr/bin/env python3
import numpy as np
import rospy


import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory, roll_variations, World, concat_lists, snake_paths


import cares_lib_ros.utils as utils
from arm_utils import run_scan, scan_cartesian, wait_goal, get_client

import math

from rospy.service import ServiceException

def transpose_lists(lists):
  return list(zip(*lists))

def main():
  rospy.init_node('arm_scan_node')
  
  distance = 0.9
  height = 1.0

  arc_params = dict(
    arc_range=(-math.pi/4, math.pi/4),
    radius=0.3,
    axes=World
  )

  arc_segments = 8
  continuous = False
  transpose = False


  client = get_client()
  branch_position = np.array([0.1, distance, height])
  home_position = branch_position - World.forward * arc_params['radius']


  ee_frame = rospy.get_param('~robot_effector_frame')

  def goto(pose):
    pose_goal = utils.create_goal_msg(pose, 0, ee_frame)
    wait_goal(client, pose_goal)


  origin_arc = PathFactory.single_arc(**arc_params, segments=64 if continuous else arc_segments)
  arcs = PathFactory.repeat_along(origin_arc, home_position, axis_range=(-0.25, 0.25), stops=12)

  if transpose:
    arcs = transpose_lists(arcs)

  if continuous:
    pathways = snake_paths(arcs)
    scan_cartesian(client, pathways, ee_frame=ee_frame)

  else:
    pathway = concat_lists(snake_paths(arcs))
    run_scan(client, pathway, "scan", ee_frame) 


  # Return to home position
  home = PathFactory.look_at(home_position, branch_position, up=World.up)
  client.send_goal(home)



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
