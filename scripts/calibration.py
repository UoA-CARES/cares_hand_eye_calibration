#!/usr/bin/env python3
import numpy as np
import rospy

import math
import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory, World, concat_lists


from camera_calibration.srv import Calibrate
from arm_utils import run_scan, get_client

def main():
  rospy.init_node('arm_calibration_node')
  client = get_client()

  camera_topic = rospy.get_param('~camera_topic', "stereo_pair")
  calibrate = rospy.ServiceProxy(f'{camera_topic}/calibrate', Calibrate)

  home_position = np.array([0.2, 0.6, 0.8])
  calib_target = np.array([0.2, 0.9, 0.8])

  pathway = [
    PathFactory.calibration_scan(home_position, calib_target, size=(3, 3, 3), up=up)
      for up in [World.up, World.right, -World.up]]
  pathway = concat_lists(pathway)

  status = run_scan(client, pathway, roll_tolerance=math.pi/2)
 
  filename = f"{status.scan_folder}/{status.session}/{status.current_scan}/capture.json"
  calibrate(scan_file=filename)

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
