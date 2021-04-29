# CARES_hand_eye_calibration
Hand Eye Calibration for Robot arms and RGB/RGBD cameras

##Dependancies
In addition to the sensor drivers you want to calibrate with please install visp handeye
```
sudo apt-get install ros-$ROS_DISTRO-visp
```

## STEREO
###STEREO AND HANDEYE
performs stereo AND hand-eye calibration at same time
```
roslaunch cares_hand_eye_calibration stereo_auto_calibration.launch
```

