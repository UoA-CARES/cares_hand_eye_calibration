# CARES_hand_eye_calibration
Hand Eye Calibration for Robot arms and RGB/RGBD cameras

## Dependancies
In addition to the sensor drivers you want to calibrate with please install visp handeye
```
sudo apt-get install ros-$ROS_DISTRO-visp
 sudo apt install ros-noetic-moveit 
cd ~/catkin_ws/src
git clone https://github.com/UoA-CARES/stereo_calibration
git clone https://github.com/UoA-CARES/mini_rig_ros
```

## STEREO
### STEREO AND HANDEYE
performs stereo AND hand-eye calibration at same time
```
roslaunch cares_hand_eye_calibration stereo_auto_calibration.launch
```

