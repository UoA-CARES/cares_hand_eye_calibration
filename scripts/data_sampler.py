import abc
import rospy
import roslib
import numpy as np
import time
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError

import open3d as o3d
import os
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cares_msgs.msg import StereoCameraInfo
import sensor_msgs.point_cloud2 as pc2

import open3d_conversions as o3d_con
import json

def image_msg_to_cv2(data):
  try:
      bridge = CvBridge()
      if data.encoding == "bayer_bggr8":
          image = bridge.imgmsg_to_cv2(data, "bgr8")
          return image
      image = bridge.imgmsg_to_cv2(data, data.encoding)
  except CvBridgeError as e:
      print(e)
  return image

def depth_msg_to_cv2(data):
    try:
      bridge = CvBridge()
      image = bridge.imgmsg_to_cv2(data, "32FC1")
      #if data.encoding == "rgb8":
      #    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
      print(e)
    return image

class DepthDataSampler(object):
  __metaclass__ = abc.ABCMeta
  def __init__(
        self,
        image_topics,
        depth_image_topic,
        camera_info_topic,
        xyzrgb_topic,
        sensor_link,
        d_roll,
        d_pitch,
        d_yaw
      ):

    self.image_topics      = image_topics
    self.depth_image_topic = depth_image_topic
    self.camera_info_topic = camera_info_topic
    self.xyzrgb_topic      = xyzrgb_topic
    
    self.depth_image = None
    self.camera_info = None
    self.xyzrgb      = None

    self.sensor_link       = sensor_link

    self.d_roll  = d_roll * math.pi/180.0
    self.d_pitch = d_pitch * math.pi/180.0
    self.d_yaw   = d_yaw * math.pi/180.0
  
    self.data_ready = False

    self.bridge = CvBridge()

  def cb_once(self, *args):
    stream_ind = args[-1]
    subs = args[-2]
    min_time = args[-3]
    
    num_subs = len(args)-3
    msgs = list(args[:num_subs])

    msg_time = msgs[0].header.stamp
    if msg_time < min_time:
      return

    for i, data in enumerate(msgs):
      if stream_ind[i] == 'image':
        self.image_msg = data
        self.image = image_msg_to_cv2(data)

      elif stream_ind[i] == 'left':
        self.left_image_msg = data
        self.left_image = image_msg_to_cv2(data)

      elif stream_ind[i] == 'right':
        self.right_image_msg = data
        self.right_image = image_msg_to_cv2(data)
          
      elif stream_ind[i] == 'depth_image':
        self.depth_image_msg = data                
        self.depth_image = depth_msg_to_cv2(data)

      elif stream_ind[i] == 'camera_info':
        self.camera_info = data

      elif stream_ind[i] == 'points':
        self.xyzrgb_msg_header = data.header
        # get opend3d point cloud from point cloud message
        if type(self.xyzrgb) == o3d.geometry.PointCloud:
            self.xyzrgb.clear()
            del self.xyzrgb
        self.xyzrgb = o3d_con.convertCloudFromRosToOpen3dI(data)
          
    # unregister subscribers
    [sub.unregister() for sub in subs]
    self.data_ready = True

  def sample_multiple_streams(
      self, 
      rgb_image=True,
      depth_image=True,
      points=True, 
      camera_info=True):
    
    subs = []
    stream_ind = []
    self.data_ready = False
    
    if rgb_image:
      for key, value in self.image_topics.items():
        sub_image_once = message_filters.Subscriber(value, Image)
        subs.append(sub_image_once)
        stream_ind.append(key) 
    
    if depth_image:
      sub_depth_image_once = message_filters.Subscriber(self.depth_image_topic, Image)
      subs.append(sub_depth_image_once)
      stream_ind.append('depth_image')
    
    if points:    
      sub_xyzrgb_once = message_filters.Subscriber(self.xyzrgb_topic, PointCloud2)
      subs.append(sub_xyzrgb_once)
      stream_ind.append('points')

    if camera_info:    
      sub_info_once = message_filters.Subscriber(self.stereo_info_topic, StereoCameraInfo)
      subs.append(sub_info_once)
      stream_ind.append('camera_info')
    
    min_time = rospy.Time.now()
    ts = message_filters.ApproximateTimeSynchronizer([sub for sub in subs],4, 4) 
    ts.registerCallback(self.cb_once, min_time, subs, stream_ind)
    
    while self.data_ready == False:
        continue
    self.data_ready = False

  def save_2d_image(self, image, file_path):
    cv2.imwrite(file_path+'_rgb.png', image)

  def save_depth_image(self, depth_image, filepath):
    cv2.imwrite(filepath+'_depth.exr', depth_image)

  def save_camera_info(self, camera_info, filepath):
    self.save_raw(str(camera_info),filepath+"_camera_info.yaml")

  def save_stereo_info(self, stereo_info, filepath):
    self.save_raw(str(stereo_info),filepath+"_stereo_info.yaml")

  def save_xyzrgb(self, xyzrgb, file_path):        
    o3d.io.write_point_cloud(file_path+'_xyzrgb.ply', xyzrgb)

  def save_raw(self,file, filepath):
    with open(filepath,"w") as f:
      f.write(file)

  @abc.abstractmethod
  def save(self, filepath):
    pass

class StereoDataSampler(DepthDataSampler):
  def __init__(self):

    image_topics = {}
    image_topics["left"]  = rospy.get_param('~left_image_topic', "stereo_pair/left/image_raw")
    image_topics["right"] = rospy.get_param('~right_image_topic', "stereo_pair/right/image_raw")

    depth_image_topic = rospy.get_param('~depth_image_topic', "stereo_pair/depth_image")
    stereo_info_topic = rospy.get_param('~camera_info_topic', "stereo_pair/stereo_info")
    xyzrgb_topic      = rospy.get_param('~xyzrgb_topic', "stereo_pair/xyzrgb")
    sensor_link       = rospy.get_param('~sensor_link', "stereo_pair/left")

    super(StereoDataSampler, self).__init__(
        image_topics=image_topics,
        depth_image_topic=depth_image_topic,
        camera_info_topic=stereo_info_topic,
        xyzrgb_topic=xyzrgb_topic,
        sensor_link=sensor_link,
        d_roll=-90,
        d_pitch=0,
        d_yaw=0
      )

    self.left_image  = None
    self.right_image = None
      
  def save(self, filepath):
    if self.left_image is not None and self.right_image is not None:
        self.save_2d_image(self.left_image, filepath+"_left")
        self.save_2d_image(self.right_image, filepath+"_right")
    if self.depth_image is not None:
        self.save_depth_image(self.depth_image, filepath)
    if self.camera_info is not None:
        self.save_stereo_info(self.camera_info, filepath)
    if self.xyzrgb is not None:
        self.save_xyzrgb(self.xyzrgb, filepath)

class DepthCameraDataSampler(DepthDataSampler):
  def __init__(self,
      image_topic,
      depth_image_topic,
      camera_info_topic,
      xyzrgb_topic,
      sensor_link,
      d_roll,
      d_pitch,
      d_yaw
    ):

    image_topics = {}
    image_topics["image"] = image_topic 

    super(DepthCameraDataSampler, self).__init__(
          image_topics=image_topics,
          depth_image_topic=depth_image_topic,
          camera_info_topic=stereo_info_topic,
          xyzrgb_topic=xyzrgb_topic,
          sensor_link=sensor_link,
          d_roll=d_roll,
          d_pitch=d_pitch,
          d_yaw=d_yaw
        )

    self.image = None

  def save(self, filepath):
    if self.iamge is not None:
        self.save_2d_image(self.image, filepath)
    if self.depth_image is not None:
        self.save_depth_image(self.depth_image, filepath)
    if self.camera_info is not None:
        self.save_stereo_info(self.camera_info, filepath)
    if self.xyzrgb is not None:
        self.save_xyzrgb(self.xyzrgb, filepath)

class RealsenseDataSampler(DepthCameraDataSampler):
  def __init__(self):
    image_topic = rospy.get_param('~image_topic', "camera/color/image_raw")

    depth_image_topic = rospy.get_param('~depth_image_topic', "camera/aligned_depth_to_color/image_raw")
    stereo_info_topic = rospy.get_param('~camera_info_topic', "camera/color/camera_info")
    xyzrgb_topic      = rospy.get_param('~xyzrgb_topic', "camera/depth/color/points")
    sensor_link       = rospy.get_param('~sensor_link', "camera_link")

    super(RealsenseDataSampler, self).__init__(
          image_topic=image_topic,
          depth_image_topic=depth_image_topic,
          camera_info_topic=stereo_info_topic,
          xyzrgb_topic=xyzrgb_topic,
          sensor_link=sensor_link,
          d_roll=0,
          d_pitch=0,
          d_yaw=90
        )      

class ZividDataSampler(DepthCameraDataSampler):
  def __init__(self):
    image_topic = rospy.get_param('~image_topic', "zivid_camera/color/image_color")

    depth_image_topic = rospy.get_param('~depth_image_topic', "zivid_camera/depth/image_raw")
    stereo_info_topic = rospy.get_param('~camera_info_topic', "zivid_camera/color/camera_info")
    xyzrgb_topic      = rospy.get_param('~xyzrgb_topic', "zivid_camera/points")
    sensor_link       = rospy.get_param('~sensor_link', "zivid_optical_frame")

    super(ZividDataSampler, self).__init__(
          image_topic=image_topic,
          depth_image_topic=depth_image_topic,
          camera_info_topic=stereo_info_topic,
          xyzrgb_topic=xyzrgb_topic,
          sensor_link=sensor_link,
          d_roll=-90,
          d_pitch=0,
          d_yaw=0
        )