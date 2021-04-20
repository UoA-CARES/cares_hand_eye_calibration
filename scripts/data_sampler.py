import abc
import rospy
import roslib
import numpy as np
import time
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
import open3d as o3d
import os
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from maara_msgs.msg import StereoCameraInfo
import sensor_msgs.point_cloud2 as pc2
import dynamic_reconfigure.client
# from zivid_camera.srv import *
import ctypes
import struct
from threading import Thread
# from pyntcloud import PyntCloud
import ros_numpy
import open3d_conversions as o3d_con
from std_srvs.srv import Trigger, TriggerResponse
import json
from zivid_camera.srv import CaptureAssistantSuggestSettings, Capture2D, Capture,CaptureAssistantSuggestSettingsRequest
def image_msg_to_cv2(data):
  try:
      print("encoding is",data.encoding)
      bridge = CvBridge()
      if data.encoding == "bayer_bggr8":
          image = bridge.imgmsg_to_cv2(data, "bgr8")
          return image
      image = bridge.imgmsg_to_cv2(data, data.encoding)
      #if data.encoding == "rgb8":
      #    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
  except CvBridgeError as e:
      print(e)
  return image

def depth_msg_to_cv2(data):
    try:
      print("encoding is",data.encoding)
      bridge = CvBridge()
      image = bridge.imgmsg_to_cv2(data, "32FC1")
      #if data.encoding == "rgb8":
      #    rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
      print(e)
    return image

class DataSampler(object):
  __metaclass__ = abc.ABCMeta
  def __init__():
    pass

  @abc.abstractmethod
  def sample_multiple_streams(self):
    pass

  @abc.abstractmethod
  def get_frame_id(self):
    pass

  @abc.abstractmethod
  def get_timestamp(self):
      pass        

  @abc.abstractmethod
  def save(self, filepath, display=False):
    pass
  #check for timestamps 
  #include timestamps in args
  def cb_once(self, *args):
    print("sakdjsalkj")
    stream_ind = args[-1]
    subs = args[-2]
    min_time = args[-3]
    
    num_subs = len(args)-3
    msgs = list(args[:num_subs])

    msg_time = msgs[0].header.stamp
    print("msg_time",msg_time, "min_time",min_time)
    if msg_time < min_time:
      print("msg_time",msg_time, "min_time",min_time)
      return

    for i, data in enumerate(msgs):
        if stream_ind[i] == 'rgb_image':
            self.rgb_msg = data
            # get rgb image from color image stream
            self.rgb = image_msg_to_cv2(data)

        elif stream_ind[i] == 'left_image':
            self.left_image_msg = data
            # get rgb image from color image stream
            self.left_image = image_msg_to_cv2(data)

        elif stream_ind[i] == 'right_image':
            self.right_image_msg = data
            # get rgb image from color image stream
            self.right_image = image_msg_to_cv2(data)
            
        elif stream_ind[i] == 'depth_image':
            self.depth_image_msg = data                
            # get depth image from depth image stream
            self.depth_image = depth_msg_to_cv2(data)

        elif stream_ind[i] == 'camera_info':
            self.camera_info = data

        elif stream_ind[i] == 'points':
            self.xyzrgb_msg_header = data.header
            # get xyzrgb point cloud from points stream
            if type(self.xyzrgb) == o3d.geometry.PointCloud:
                self.xyzrgb.clear()
                del self.xyzrgb
            self.xyzrgb = self.extract_xyzrgb(data)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.xyzrgb[:,:3])
            pcd.colors = o3d.utility.Vector3dVector(np.divide(self.xyzrgb[:,3:],255.0))
            del self.xyzrgb
            self.xyzrgb = pcd
    [sub.unregister() for sub in subs]

    self.data_ready = True

  def save_2d_image(self, image, file_path, display=False):
    if display:
        cv2.imshow("2D Color Image", image)
        cv2.waitKey(1)

    cv2.imwrite(file_path+'_rgb.png', image)

  def save_depth_image(self, depth_image, filepath, display=False):
    if display:
        cv2.imshow("Depth Image", depth_image)
        cv2.waitKey(1)

    # Need to update this to save proper floating point values
    cv2.imwrite(filepath+'_depth.exr', depth_image)

  def save_camera_info(self, camera_info, filepath):
    pass

  def save_stereo_info(self, stereo_info, filepath):
    self.save_raw(str(stereo_info),filepath+"_stereo_info.yaml")

  def save_xyzrgb(self, xyzrgb, file_path, display=False):        
    o3d.io.write_point_cloud(file_path+'_xyzrgb.ply', xyzrgb)

  def save_raw(self,file, filepath):
    with open(filepath,"w") as f:
      f.write(file)

class DataSamplerDepthSensor(DataSampler):
    __metaclass__  = abc.ABCMeta
    def __init__(
            self, 
            image_topic,  
            depth_image_topic, 
            camera_info_topic, 
            xyzrgb_topic,
            sensor_link,
            d_roll,
            d_pitch,
            d_yaw,
            save_rgb=False, 
            save_depth_image=False,
            save_camera_info=False, 
            save_xyzrgb=False
        ):
        
        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.camera_info_topic = camera_info_topic
        self.xyzrgb_topic = xyzrgb_topic

        self.do_save_rgb = save_rgb
        self.do_save_depth_image = save_depth_image
        self.do_save_camera_info = save_camera_info
        self.do_save_xyzrgb = save_xyzrgb

        self.sensor_link = sensor_link

        self.d_roll  = d_roll * math.pi / 180.0 
        self.d_pitch = d_pitch * math.pi / 180.0 
        self.d_yaw   = d_yaw * math.pi / 180.0 
        
        self.rgb = None
        self.depth_image = None
        self.camera_info = None
        self.xyzrgb = None

        self.rgb_msg = None
        self.depth_image_msg = None
        self.xyzrgb_msg_header = None

        self.data_ready = False

        self.bridge = CvBridge()

    def get_frame_id(self):
      return self.xyzrgb_msg_header.frame_id

    def get_timestamp(self):
      return self.xyzrgb_msg_header.stamp        

    @abc.abstractmethod
    def extract_xyzrgb(self, data):
        pass

    @abc.abstractmethod
    def capture(self, capture2d=False, capture3d=False):
        pass

    def sample_multiple_streams(
            self, 
            rgb_image=True,  
            depth_image=True, 
            points=True, 
            camera_info=True, 
            display=False
        ):
        
        subs = []
        stream_ind = []
        
        if rgb_image:
          sub_image_once = None
          sub_image_once = message_filters.Subscriber(self.image_topic, Image)
          subs.append(sub_image_once)
          stream_ind.append('rgb_image') 
          capture2d = True
        
        if depth_image and self.depth_image_topic is not None:
          sub_depth_image_once = None
          sub_depth_image_once = message_filters.Subscriber(self.depth_image_topic, Image)

          subs.append(sub_depth_image_once)
          stream_ind.append('depth_image')
          capture3d = True
        
        if camera_info:    
          sub_info_once = None
          sub_info_once = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
          subs.append(sub_info_once)
          stream_ind.append('camera_info')
        
        if points:    
          sub_xyzrgb_once = None
          sub_xyzrgb_once = message_filters.Subscriber(self.xyzrgb_topic, PointCloud2)
          subs.append(sub_xyzrgb_once)
          stream_ind.append('points')
          capture3d = True
        
        min_time = rospy.Time.now()
        ts = message_filters.ApproximateTimeSynchronizer([sub for sub in subs],4, 4) 
        ts.registerCallback(self.cb_once,min_time, subs, stream_ind)

        self.capture(capture2d=capture2d, capture3d=capture3d)
        
        while self.data_ready == False:
            continue
        self.data_ready = False

    def save(self, filepath, display=False):
      if self.do_save_rgb:
          self.save_2d_image(self.rgb, filepath, display=display)
      if self.do_save_depth_image:
          self.save_depth_image(self.depth_image, filepath, display=display)
      if self.do_save_camera_info:
          self.save_camera_info(self.camera_info, filepath)
      if self.do_save_xyzrgb:
          self.save_xyzrgb(self.xyzrgb, filepath, display=display)


class DataSamplerRealSense(DataSamplerDepthSensor):
    def __init__(
            self,
            save_rgb=False, 
            save_depth_image=False,
            save_camera_info=False,
            save_xyzrgb=False,
        ):
        
        super(DataSamplerRealSense, self).__init__(
            image_topic="camera/color/image_raw", 
            depth_image_topic="camera/aligned_depth_to_color/image_raw",
            camera_info_topic="camera/color/camera_info",
            xyzrgb_topic="camera/depth/color/points",
            sensor_link="camera_link",
            d_roll=0,
            d_pitch=0,
            d_yaw=90, 
            save_rgb=save_rgb,
            save_depth_image=save_depth_image, 
            save_camera_info=save_camera_info, 
            save_xyzrgb=save_xyzrgb
        )

    def capture(self, capture2d=False, capture3d=False):
        pass

    def decode_pointcloud2_msg(self, pc):

        xyzrgb = []
        for p in pc:
            test = p[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            xyzrgb.append([p[0], p[1], p[2], r, g, b])  # x,y,z can be retrieved from the p[0],p[1],p[2]
            # print(p[0], p[1], p[2], r, g, b) # prints x, y, z and r, g, b values in the 0-255 range

        return xyzrgb

    def extract_xyzrgb(self, data):
        # pc2_msg into numpy array of xyzrgb
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb"))
        pc = list(pc)
        xyzrgb = self.decode_pointcloud2_msg(pc)
        xyzrgb = np.array(xyzrgb)
        return xyzrgb

class DataSamplerZivid(DataSamplerDepthSensor):
    def __init__(
            self,
            save_rgb=False, 
            save_depth_image=False,
            save_camera_info=False,
            save_xyzrgb=False,
        ):

        super(DataSamplerZivid, self).__init__(
            image_topic="zivid_camera/color/image_color", 
            depth_image_topic="zivid_camera/depth/image_raw",
            camera_info_topic="zivid_camera/color/camera_info",
            xyzrgb_topic="zivid_camera/points",
            sensor_link="zivid_optical_frame",
            d_roll= -90,
            d_pitch=0,
            d_yaw=0,
            save_rgb=save_rgb, 
            save_depth_image=save_depth_image, 
            save_camera_info=save_camera_info, 
            save_xyzrgb=save_xyzrgb
        )

        ca_suggest_settings_service = "zivid_camera/capture_assistant/suggest_settings"

        rospy.wait_for_service(ca_suggest_settings_service, 30.0)

        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )

        rospy.wait_for_service("zivid_camera/capture", 30.0)

        rospy.wait_for_service("zivid_camera/capture_2d", 30.0)

        self.capture_2D_service = rospy.ServiceProxy(
            "zivid_camera/capture_2d", Capture2D
        )

        self.capture_3D_service = rospy.ServiceProxy(
            "zivid_camera/capture", Capture
        )

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(1.20)
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self, capture2d=False, capture3d=False):
        self.capture_assistant_suggest_settings()
        if capture3d:
            self.capture_3D_service()
        elif capture2d:
            self.capture_2D_service()

    def extract_xyzrgb(self, data):
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pc = pc.flatten()
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        xyzrgb=np.zeros((pc.shape[0],6))
        xyzrgb[:,0]=pc['x']
        xyzrgb[:,1]=pc['y']
        xyzrgb[:,2]=pc['z']
        xyzrgb[:,3]=pc['r']
        xyzrgb[:,4]=pc['g']
        xyzrgb[:,5]=pc['b']
        xyzrgb = xyzrgb[~np.isnan(xyzrgb).any(axis=1)]        
        return xyzrgb

class DataSamplerStereoDepth(DataSampler):
    def __init__(
            self,
            save_rgb=False, 
            save_depth_image=False,
            save_camera_info=False,
            save_xyzrgb=False
        ):
        self.left_image_topic="/camera_array/cam1/image_raw" 
        self.right_image_topic="/camera_array/cam2/image_raw" 
        
        #self.depth_image_topic="/stereo/depth"
        self.depth_image_topic="/camera_array/rgbd/depth"
        self.stereo_info_topic="/camera_array/stereo1/stereo_info_full"
        # self.stereo_info_topic="/rectified_image/stereo_info"
        #self.xyzrgb_topic="/stereo/points"
        self.xyzrgb_topic="/camera_array/rgbd/points"

        self.sensor_link="stereo1/left"
        
        self.d_roll = -90 * math.pi/180.0
        self.d_pitch = 0
        self.d_yaw = 0

        self.do_save_rgb = save_rgb
        self.do_save_depth_image = save_depth_image 
        self.do_save_camera_info = save_camera_info 
        self.do_save_xyzrgb = save_xyzrgb

        self.left_rgb = None
        self.right_rgb = None
        self.depth_image = None
        self.camera_info = None
        self.xyzrgb = None

        self.left_msg  = None
        self.right_msg = None
        self.depth_image_msg = None
        self.xyzrgb_msg_header = None
        self.stereo_info = None

        self.data_ready = False

        self.bridge = CvBridge()

        self.data_ready = False

    def get_frame_id(self):
      return self.sensor_link

    def get_timestamp(self):
      return self.xyzrgb_msg_header.stamp        

    def capture(self, capture2d=False, capture3d=False):
      pass

    def extract_xyzrgb(self, data):
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        pc = pc.flatten()
        pc = ros_numpy.point_cloud2.split_rgb_field(pc)
        xyzrgb=np.zeros((pc.shape[0],6))
        xyzrgb[:,0]=pc['x']
        xyzrgb[:,1]=pc['y']
        xyzrgb[:,2]=pc['z']
        xyzrgb[:,3]=pc['r']
        xyzrgb[:,4]=pc['g']
        xyzrgb[:,5]=pc['b']
        xyzrgb = xyzrgb[~np.isnan(xyzrgb).any(axis=1)]        
        return xyzrgb

    def sample_multiple_streams(
          self, 
          rgb_image=False,  
          depth_image=False, 
          points=False, 
          camera_info=False, 
          display=False
      ):
      
      subs = []
      stream_ind = []
      if rgb_image:
          sub_left_image_once = None
          sub_left_image_once = message_filters.Subscriber(self.left_image_topic, Image)
          subs.append(sub_left_image_once)
          print("got left_image")
          stream_ind.append('left_image') 

          sub_right_image_once = None
          sub_right_image_once = message_filters.Subscriber(self.right_image_topic, Image)
          subs.append(sub_right_image_once)
          print("got left_image")
          stream_ind.append('right_image') 
      
      if depth_image and self.depth_image_topic is not None:
          sub_depth_image_once = None
          sub_depth_image_once = message_filters.Subscriber(self.depth_image_topic, Image)

          subs.append(sub_depth_image_once)
          print("got depth")
          stream_ind.append('depth_image')
      
      if camera_info:    
          sub_info_once = None
          sub_info_once = message_filters.Subscriber(self.stereo_info_topic, StereoCameraInfo)
          subs.append(sub_info_once)
          print("got camera_info")
          stream_ind.append('camera_info')
      
      if points:    
          sub_xyzrgb_once = None
          sub_xyzrgb_once = message_filters.Subscriber(self.xyzrgb_topic, PointCloud2)
          subs.append(sub_xyzrgb_once)
          print("got pointcloud")
          stream_ind.append('points')
      
      min_time = rospy.Time.now()
      ts = message_filters.ApproximateTimeSynchronizer([sub for sub in subs], 4, 4) 
      ts.registerCallback(self.cb_once, min_time, subs, stream_ind)

      self.capture(capture2d=True, capture3d=True)
      
      while self.data_ready == False:
          continue
      self.data_ready = False

    def save(self, filepath, display=False):
      if self.do_save_rgb:
          self.save_2d_image(self.left_image, filepath+"left", display=display)
          self.save_2d_image(self.right_image, filepath+"right", display=display)
      if self.do_save_depth_image:
          self.save_depth_image(self.depth_image, filepath, display=display)
      if self.do_save_camera_info:
          self.save_stereo_info(self.camera_info, filepath)
      if self.do_save_xyzrgb:
          self.save_xyzrgb(self.xyzrgb, filepath, display=display)

####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
####################################################################################################################################

class DataSamplerMicrowave():
    def __init__(
            self,
            save_microwave=False,
        ):
        # These need to be pulled up to the top level
        capture_service_topic = "microwave/capture"
        rospy.wait_for_service(capture_service_topic, 30.0)

        self.capture_service = rospy.ServiceProxy(capture_service_topic, Trigger)

        self.microwave_topic  = "" 
        
        self.do_save_microwave = save_microwave

        self.microwave_data = None

        self.microwave_msg = None

        self.sensor_link = "ee_link"

    def capture(self):
        response = TriggerRequest()
        self.capture_service.call(response)
        return response.success

    def convert_microwave_msg_to_data(self, data):
        return data

    def sample_multiple_streams(
            self, 
            microwave_data=True,
            display=False
        ):
        
        if self.capture():
            self.microwave_msg  = rospy.wait_for_message(self.microwave_topic, MicrowaveData)
            self.microwave_data = self.convert_microwave_msg_to_data(self.microwave_msg)
        else:
            print("Microwave failed to trigger")

    def save(self, filepath, display=False):
        # if self.do_save_microwave:
            # cv2.imwrite(filepath+'_microwave.txt', self.microwave_data)
        pass
