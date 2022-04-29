import rospy
import tf2_ros
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from handeye_calibration import HandeyeCalibration
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

import cv2
from cv_bridge import CvBridge

from cares_msgs.srv import CalibrationService, ArucoDetect
import cares_lib_ros.utils as utils
from cares_lib_ros.data_sampler import DataSamplerFactory, DepthDataSampler, StereoDataSampler

class Calibrator(object):
    def __init__(self, image_sampler, target_marker_id):
        self.image_sampler = image_sampler
        self.image_list = []

        # Marker we are tracking to calibrate on
        self.target_marker_id = target_marker_id

        # Setup Charuco detection service
        self.aruco_detect = rospy.ServiceProxy('aruco_detector', ArucoDetect)

        self.eye_on_hand = rospy.get_param('~eye_on_hand', True)
        self.robot_effector_frame = rospy.get_param('~robot_effector_frame')
        self.robot_base_frame     = rospy.get_param('~robot_base_frame')

        self.bridge = CvBridge()

    def calibrate(self, filepath):
        print("Running Hand-Eye Calibrating")
        
        # Setup hand eye calibrator
        hand_eye_calibrator = HandeyeCalibrator()

        bridge = CvBridge()
        for i in range(len(self.image_list)):
            # Get transform from charuco detection service
            aruco_transforms = self.detect_aruco(self.image_list[i])

            if self.target_marker_id in aruco_transforms.ids:
                index = aruco_transforms.ids.index(self.target_marker_id)
                transform = aruco_transforms.transforms[index]
                sample = {'robot':self.image_list[i][3], 'optical':transform}

                # print(sample)
                hand_eye_calibrator.collect_samples(sample)
            else:
                print("Marker not detected")

        # Compute the calibration
        calibration = hand_eye_calibrator.compute_calibration()
        hand_to_optical_transform  = calibration.transformation
        hand_to_body_transform  = self.rotate_to_body_frame(hand_to_optical_transform)
        calibration.transformation = hand_to_body_transform

        print("Hand-eye calibration:")
        print(calibration.to_dict())

        if calibration is not None:
            calibration.to_file(filepath[:-1], self.image_sampler.sensor_name+"_handeye_calibration")

    def collect_samples(self, file_name, tf_buffer):
        self.image_sampler.sample_multiple_streams()

        sensor_timestamp = self.image_sampler.time_stamp

        if self.eye_on_hand:
            transform = tf_buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, sensor_timestamp, rospy.Duration(1.0))
        else:
            transform = tf_buffer.lookup_transform(self.robot_effector_frame, self.robot_base_frame, sensor_timestamp, rospy.Duration(1.0))

        self.image_sampler.save(file_name)
        utils.save_transform(file_name+"_transforms.yaml", transform)

        self.image_list.append(self.sample_data() + [transform, file_name])

    # The stereo calibration produces a calibration to the optical frame, we need to the body frame
    # This is just a rigid rotation around the optical frame
    # See body and optical frame standard definitions - https://www.ros.org/reps/rep-0103.html
    def rotate_to_body_frame(self, optical_frame_transform):
        # Invert conversion from body to opitcal frame orientation
        optical_frame_rotation = utils.quaternion_to_array(optical_frame_transform.transform.rotation)
        optical_to_body = quaternion_from_euler(utils.deg_rad(-90), utils.deg_rad(0), utils.deg_rad(-90))
        optical_to_body[3] = -optical_to_body[3]
        body_frame_rotation = quaternion_multiply(optical_frame_rotation, optical_to_body)    
        
        body_frame_transform = TransformStamped(transform=optical_frame_transform.transform)
        body_frame_transform.transform.rotation.x = float(body_frame_rotation[0])
        body_frame_transform.transform.rotation.y = float(body_frame_rotation[1])
        body_frame_transform.transform.rotation.z = float(body_frame_rotation[2])
        body_frame_transform.transform.rotation.w = float(body_frame_rotation[3])
        return body_frame_transform

    def load_data(self, filepath):
        pass

    def detect_aruco(self, image_data):
        pass

    def sample_data(self):
        pass

class StereoCalibrator(Calibrator):
    def __init__(self, image_sampler, target_marker_id):
        super(StereoCalibrator, self).__init__(
            image_sampler=image_sampler,
            target_marker_id=target_marker_id
            )

    def calibrate(self, filepath):
        print("Running Stereo Calibration Service")
        calibration_service_name = 'stereo_calibration'
        calibration_service = rospy.ServiceProxy(calibration_service_name, CalibrationService)
        result = calibration_service(filepath)
        stereo_info = result.stereo_info
        
        print("Stereo Information:")
        print(stereo_info)

        print("Rectifying Images")
        # Rectify the images before sending them to the marker detector service
        images_left  = [img[0] for img in self.image_list]
        images_right = [img[1] for img in self.image_list]
        images_left_rectified, images_right_rectified = utils.rectify_images(images_left, images_right, stereo_info)

        for i in range(len(self.image_list)):
            self.image_list[i][0] = images_left_rectified[i]
            self.image_list[i][1] = images_right_rectified[i]
            self.image_list[i][2] = stereo_info

        super(StereoCalibrator, self).calibrate(filepath)

    def load_data(self, filepath):
        self.image_list = []
        left_image, files  = utils.loadImages(filepath+"*left_image_color.png")
        right_image, files = utils.loadImages(filepath+"*right_image_color.png")
        transforms, files  = utils.load_transforms(filepath+"*transforms.yaml")
        assert len(left_image) == len(right_image) == len(transforms)
        for i in range(len(left_image)):
            file_name = filepath+str(i)+"_"
            self.image_list.append([left_image[i], right_image[i], None, transforms[i], file_name])

    def detect_aruco(self, image_data):
        msg_left_rectified  = self.bridge.cv2_to_imgmsg(image_data[0], encoding="passthrough")
        msg_right_rectified = self.bridge.cv2_to_imgmsg(image_data[1], encoding="passthrough")
        stereo_info = image_data[2]
        # return self.aruco_detect(msg_left_rectified, msg_right_rectified, stereo_info, None, None, None)
        return self.aruco_detect(msg_left_rectified, msg_right_rectified, stereo_info, msg_left_rectified, None, stereo_info.left_info)

    def sample_data(self):
        return [self.image_sampler.left_image, self.image_sampler.right_image, None]

class DepthCalibrator(Calibrator):
    def __init__(self, image_sampler, target_marker_id):
        super(DepthCalibrator, self).__init__(
            image_sampler=image_sampler,
            target_marker_id=target_marker_id
            )

    def load_data(self, filepath):
        # Fill in the file loading method to test with file loading on depth data
        self.image_list = []
        images, _       = utils.loadImages(filepath+"*image_color.png")
        depth_images, _ = utils.loadImages(filepath+"*depth.tif")
        transforms, _   = utils.load_transforms(filepath+"*transforms.yaml")
        camera_info     = utils.load_camerainfo(filepath+"0_camera_info.yaml")

        assert len(images) == len(depth_images) == len(transforms)
        for i in range(len(images)):
            file_name = filepath+str(i)+"_"
            self.image_list.append([images[i], depth_images[i], camera_info, transforms[i], file_name])

    def detect_aruco(self, image_data):
        msg_image       = self.bridge.cv2_to_imgmsg(image_data[0], encoding="passthrough")
        msg_depth_image = self.bridge.cv2_to_imgmsg(image_data[1], encoding="passthrough")
        camera_info = image_data[2]
        return self.aruco_detect(None, None, None, msg_image, msg_depth_image, camera_info)

    def sample_data(self):
        return [self.image_sampler.image, self.image_sampler.depth_image, self.image_sampler.camera_info]

class CalibratorFactory(object):
    def __init__(self):
        pass

    @staticmethod
    def create_calibrator(sensor, target_marker_id):
        sensor_sampler = DataSamplerFactory.create_datasampler(sensor)
        if isinstance(sensor_sampler, DepthDataSampler):
            return DepthCalibrator(sensor_sampler, target_marker_id)
        elif isinstance(sensor_sampler, StereoDataSampler):
            return StereoCalibrator(sensor_sampler, target_marker_id)
        return None

class HandeyeCalibrator(object):
    """
    Connects tf and ViSP hand2eye to provide an interactive mean of calibration.
    """

    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    def __init__(self):
        self.eye_on_hand = rospy.get_param('~eye_on_hand', True)
        """
        if false, it is a eye-on-base calibration
        :type: bool
        """

        # tf names
        self.robot_effector_frame = rospy.get_param('~robot_effector_frame', 'tool0')
        """
        robot tool tf name
        :type: string
        """
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'base_link')
        """
        robot base tf name
        :type: str
        """
        self.tracking_base_frame = rospy.get_param('~tracking_base_frame', 'optical_origin')
        """
        tracking system tf name
        :type: str
        """
        self.tracking_marker_frame = rospy.get_param('~tracking_marker_frame', 'optical_target')
        """
        tracked object tf name
        :type: str
        """

        # tf structures
        self.tfBuffer = tf2_ros.Buffer()
        """
        used to get transforms to build each sample
        :type: tf2_ros.Buffer
        """
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        """
        used to get transforms to build each sample
        :type: tf2_ros.TransformListener
        """
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        """
        used to publish the calibration after saving it
        :type: tf.TransformBroadcaster
        """

        # internal input data
        self.samples = []
        """
        list of acquired samples
        Each sample is a dictionary going from 'rob' and 'opt' to the relative sampled transform in tf tuple format.
        :type: list[dict[str, ((float, float, float), (float, float, float, float))]]
        """

        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)
        """
        proxy to a ViSP hand2eye calibration service
        :type: rospy.ServiceProxy
        """

    def _wait_for_tf_init(self):
        """
        Waits until all needed frames are present in tf.
        :rtype: None
        """
        self.tfBuffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, rospy.Time(0), rospy.Duration(10))
        self.tfBuffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame, rospy.Time(0), rospy.Duration(10))

    def _get_transforms(self, time=None):
        """
        Samples the transforms at the given time.
        :param time: sampling time (now if None)
        :type time: None|rospy.Time
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        if time is None:
            #time = rospy.Time.now()
            time = rospy.Time(0)

        try:
            # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
            if self.eye_on_hand:
                rob = self.tfBuffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, time, rospy.Duration(4))
            else:
                rob = self.tfBuffer.lookup_transform(self.robot_effector_frame, self.robot_base_frame, time, rospy.Duration(4))
            opt = self.tfBuffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame, time, rospy.Duration(4))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Required transforms not found")
            return None
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.
        :rtype: None
        """
        rospy.loginfo("Taking a sample...")
        transforms = self._get_transforms()
        if transforms is not None:
            rospy.loginfo("Got a sample")
            self.samples.append(transforms)
            return transforms
        return None

    def collect_samples(self, transforms):
        """
        Samples the transformations and appends the sample to the list.
        :rtype: None
        """
        if transforms is not None:
            rospy.loginfo("Got a sample")
            self.samples.append(transforms)
            return transforms
        return None

    def remove_sample(self, index):
        """
        Removes a sample from the list.
        :type index: int
        :rtype: None
        """
        if 0 <= index < len(self.samples):
            del self.samples[index]

    def get_visp_samples(self):
        """
        Returns the sample list as a TransformArray.
        :rtype: visp_hand2eye_calibration.msg.TransformArray
        """
        hand_world_samples = TransformArray()
        # hand_world_samples.header.frame_id = self.robot_base_frame
        hand_world_samples.header.frame_id = self.tracking_base_frame
        # DONTFIXME: yes, I know, it should be like the line above.
        # thing is, otherwise the results of the calibration are wrong. don't ask me why

        camera_marker_samples = TransformArray()
        camera_marker_samples.header.frame_id = self.tracking_base_frame

        for s in self.samples:
            camera_marker_samples.transforms.append(s['optical'].transform)
            hand_world_samples.transforms.append(s['robot'].transform)

        return hand_world_samples, camera_marker_samples

    def compute_calibration(self):
        """
        Computes the calibration through the ViSP service and returns it.
        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if len(self.samples) < HandeyeCalibrator.MIN_SAMPLES:
            rospy.logwarn("{} more samples needed! Not computing the calibration".format(HandeyeCalibrator.MIN_SAMPLES - len(self.samples)))
            return

        # Update data
        hand_world_samples, camera_marker_samples = self.get_visp_samples()

        if len(hand_world_samples.transforms) != len(camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        rospy.loginfo("Computing from %g poses..." % len(self.samples))

        try:
            result = self.calibrate(camera_marker_samples, hand_world_samples)
            rospy.loginfo("Computed calibration: {}".format(str(result)))
            transl = result.effector_camera.translation
            rot = result.effector_camera.rotation
            result_tuple = ((transl.x, transl.y, transl.z),
                            (rot.x, rot.y, rot.z, rot.w))

            ret = HandeyeCalibration(self.eye_on_hand,
                                     self.robot_base_frame,
                                     self.robot_effector_frame,
                                     self.tracking_base_frame,
                                     result_tuple)

            return ret

        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: " + str(ex))
            return None
