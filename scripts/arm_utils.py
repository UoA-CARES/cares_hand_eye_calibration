import rospy
from actionlib_msgs.msg import GoalStatus

import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import roll_variations

import actionlib
from platform_msgs.msg import PlatformGoalAction
from actionlib_msgs.msg import GoalStatus

from scan_capture.srv import Capture
from scan_capture.msg import ScanStatus, CaptureStatus
# from scan_capture import cfg

from dynamic_reconfigure.client import Client

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



import math


messages = {result : key for key, result in GoalStatus.__dict__.items() 
    if not key.startswith("_") and isinstance(result, int)}


def wait_goal(platform_client, pose_goal):
    platform_client.send_goal(pose_goal)
    platform_client.wait_for_result()
    
    #handle result feedback - fail, success, etc
    result = platform_client.get_state()
    rospy.loginfo(f"move_path: {result} {messages[result]}")

    return result


def move_path(platform_client, pathway, ee_frame, roll_tolerance=0):
  for pose in pathway:
    if roll_tolerance > 0:
      poses = roll_variations(pose, angle_range=roll_tolerance, stops=16)
      pose_goal = utils.create_multi_goal_msg(poses, 0, ee_frame)

    else:
      pose_goal = utils.create_goal_msg(pose, 0, ee_frame)

    rospy.loginfo(pose)
    result = wait_goal(platform_client, pose_goal)
    yield (pose, result)

    if rospy.is_shutdown():
        return


def get_client():
  # Creates the SimpleActionClient, passing the type of the action
  platform_server = rospy.get_param('~platform_server', 'server')
  platform_client = actionlib.SimpleActionClient(
      platform_server, PlatformGoalAction)

  print("Waiting for platform server - "+platform_server)
  platform_client.wait_for_server()
  return platform_client


class CaptureClient(object):

  def __init__(self):
    self.camera_topic = rospy.get_param('~camera_topic', "stereo_pair")
    self.capture = rospy.ServiceProxy(f'{self.camera_topic}/capture_command', Capture)
    self.reconf = Client(f'{self.camera_topic}/capture_images', timeout=1)

  def capture_status(self):
    return rospy.wait_for_message(f"{self.camera_topic}/capture_status", CaptureStatus)


  def scan_folder(self):
    status = self.capture_status()
    return f"{status.scan_folder}/{status.session}/{status.current_scan}/capture.json"


  def single(self):
      self.capture(source="scan", action="single")
      rospy.wait_for_message(f"{self.camera_topic}/scan_status", ScanStatus)

  
  def enable(self, enabled=True):

    if enabled:
      self.capture(source="scan", action="start")
    else:
      self.capture(source="scan", action="stop")


  def new_folder(self, scan_name=None):
    if scan_name is not None:
        self.reconf.update_configuration(dict(folder_prefix=scan_name))

    return self.capture(source="scan", action="new")



def run_scan(platform_client, pathway, scan_name=None, ee_frame="flange", roll_tolerance=0, pause_duration=0.1):

  capture = CaptureClient()
  capture.new_folder(scan_name)

  for _, result in move_path(platform_client, pathway, ee_frame, roll_tolerance):
    if result == GoalStatus.SUCCEEDED:
      rospy.loginfo("move succeeded, capturing frame")
      rospy.sleep(pause_duration)

      capture.single()


  filename = capture.scan_folder()
  capture.new_folder()

  rospy.loginfo(f"captured {filename}")
  return filename



def scan_cartesian(platform_client, pathways, scan_name=None, ee_frame="flange"):
  def cartesian_pathway(pathway):
      goal = utils.create_multi_goal_msg(pathway, 0, ee_frame, cartesian_path=True)
      wait_goal(platform_client, goal)


  capture = CaptureClient()
  capture.new_folder(scan_name)
  
  filename = capture.scan_folder()


  for pathway in pathways:
    capture.enable(True)
    cartesian_pathway(pathway)
    capture.enable(False)

  
  capture.new_folder()
  return filename