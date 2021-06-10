#!/usr/bin/env python3

from os.path import expanduser

from structs.struct import struct
from shared.calibration import calibrate_images

import rospy
import os
import cv2

import cares_lib_ros.utils as utils
from cares_lib_ros.data_sampler import StereoDataSampler

from shared.arm_scan import acquire_images

def to_gray(image):
  if image.ndim == 3 and image.shape[2] == 3:
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

  assert image.ndim == 2,\
    f"expected grayscale or bgr image, got shape {image.shape}"
  return image  


def main():
  rospy.init_node("multical_auto_calibration")

  file_path = expanduser(rospy.get_param('~file_path', None))
  files, images = acquire_images(file_path)

  left = [to_gray(i) for i in images["left"]]
  right = [to_gray(i) for i in images["right"]]

  camera_images = struct(
    image_path = file_path, 
    cameras    = ["left", "right"], 
    images      = [left, right],
    image_names = files['image_names'], 
    filenames   = [files["left"], files["right"]]
  )

  transforms = [ utils.rt_to_homog(*utils.transform_to_rt(t.transform)) 
    for t in  images["transforms"]]

  calibrate_images(camera_images, transforms,
    board_config=rospy.get_param('~board_config', None))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

