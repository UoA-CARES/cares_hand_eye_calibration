from os import path
from multical.io.logging import setup_logging
from multical.optimization import HandEyeCalibration

from multical.transform import matrix, qtvec
from multical.io import info
from structs.numpy import shape

from structs.struct import struct, transpose_lists

from multical import workspace, io
from multical.config import find_board_config, initialise_with_images

import cv2


def rectify_pair(calib, l="left", r="right"):
  left = calib.cameras[l]
  right = calib.cameras[r]

  r, t = matrix.split(calib.camera_poses.relative(r, l))
  r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(left.intrinsic, left.dist, right.intrinsic, right.dist, left.image_size, r, t)
  return struct(r1=r1, r2=r2, p1=p1, p2=p2, q=q, roi1=roi1, roi2=roi2)


def to_quat(m):
  return qtvec.split(qtvec.from_matrix(m))

def report_poses(hand_eye):

  for k, cam_wrt_gripper in hand_eye.cameras_wrt_gripper.items():
    info(k)
    info(cam_wrt_gripper)
    q, t = to_quat(cam_wrt_gripper)
    info(f"q(xyzw) = {q}, t = {t}")


def calibrate_images(file_path, files, scan, 
  camera_names=["left", "right"], board_config=None):
  
  camera_images = struct(
    image_path =file_path, 
    cameras    =camera_names, 
    image_names=files['image_names'], 
    images     =[scan[k] for k in camera_names],
    filenames  =[files[k] for k in camera_names])

  ws = workspace.Workspace(file_path, "hand-eye")

  log_file=path.join(file_path, "log.txt")
  setup_logging("DEBUG", [ws.log_handler], log_file)

  boards = find_board_config(file_path, board_file=board_config)

  initialise_with_images(ws, boards, camera_images)
  ws.calibrate("initial", boards=True)

  ws.dump()
  ws.export(master=camera_names[0])

  calib = ws.latest_calibration
  hand_eye = HandEyeCalibration.initialise(calib, scan["transforms"])
  hand_eye.report_error("hand-eye init")
  ws.push_calibration("hand-eye initialised", hand_eye.calib)
  report_poses(hand_eye)

  hand_eye = hand_eye.bundle_adjust()
  hand_eye.report_error("reprojection hand-eye")
  ws.push_calibration("hand-eye reproj", hand_eye.calib)
  report_poses(hand_eye)
     
  ws.dump()


