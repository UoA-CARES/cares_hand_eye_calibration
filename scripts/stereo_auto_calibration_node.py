#!/usr/bin/env python3
from cares_hand_eye_calibration.scripts.shared.arm_scan import acquire_images
import rospy

import numpy as np
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from cares_msgs.srv import CalibrationService, ArucoDetect

import cares_lib_ros.utils as utils

from os.path import expanduser

def detect_marker(stereo_info, marker_id = 11):
  bridge = CvBridge()
  aruco_detect = rospy.ServiceProxy('aruco_detector', ArucoDetect)

  def detect(image_pair):
    msg_left_rectified  = bridge.cv2_to_imgmsg(image_pair[0], encoding="passthrough")
    msg_right_rectified = bridge.cv2_to_imgmsg(image_pair[1], encoding="passthrough")

          # Get transform from charuco detection service
    aruco_transforms = aruco_detect(msg_left_rectified, msg_right_rectified, stereo_info)
    if marker_id in aruco_transforms.ids:
        index = aruco_transforms.ids.index(marker_id)
        return aruco_transforms.transforms[index]

  return detect



def main():
    rospy.init_node('stereo_auto_calibration_node')

    file_path = expanduser(rospy.get_param('~file_path', None))
    files, images = acquire_images(file_path)


    # Run calibration service
    print("Running Stereo Calibration Service")
    calibration_service_name = 'stereo_calibration'
    calibration_service = rospy.ServiceProxy(calibration_service_name, CalibrationService)
    result = calibration_service(file_path)
    stereo_info = result.stereo_info
    
    print("Stereo Information:")
    print(stereo_info)
    
    # Rectify the images before sending them to the marker detector service
    images_left, images_right, transforms  = images["left"], images["right"], images["transforms"]
    images_rectified = utils.rectify_images(images_left, images_right, stereo_info)
    
    detect = detect_marker(stereo_info)
    marker_transforms = [detect(pair) for pair in zip (*images_rectified)]

    # Compute the calibration
    
    print("Done calibration")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
