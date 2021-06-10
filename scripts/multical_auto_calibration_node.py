#!/usr/bin/env python3

from os.path import expanduser
from shared.calibration import calibrate_images

import rospy
import os

import cares_lib_ros.utils as utils
from cares_lib_ros.data_sampler import StereoDataSampler

from shared.arm_scan import acquire_images

def main():
  rospy.init_node("multical_auto_calibration")

  file_path = expanduser(rospy.get_param('~file_path', None))
  files, images = acquire_images(file_path)

  calibrate_images(file_path, files, images, 
    board_config=rospy.get_param('~board_config', None))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
