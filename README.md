# CARES_hand_eye_calibration
Hand Eye Calibration for Robot arms and RGB/RGBD cameras. 

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Base package dependencies

```
1) Ubuntu 20.04 - ROS Noetic - requires python 3 support

2) Pull master version of cares_msgs
   a) cd ~/catkin_ws/src
   b) git clone https://github.com/UoA-CARES/cares_msgs.git

3) Pull master version of platform_msgs
	a) cd ~/catkin_ws/src
	b) git clone https://github.com/maraatech/platform_msgs

4) Pull master version of cares_lib_ros
	a) cd ~/catkin_ws/src
	b) git clone https://github.com/UoA-CARES/cares_lib_ros.git

4) Install VISP library (future will remove this dependancy to utilise opencv hand-eye calibration library)
	a) https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html
	b) cd ~/catkin_ws/src
	c) git clone https://github.com/lagadic/vision_visp.git
```

### Calabration and Marker Services
Hand eye calibration relies on other packages/services to conduct the stereo calibration and marker detection.\
These can be replaced with different vairations by changing the respective service topics in the launch files.\
The two below are the ones specifically designed for this package but are not the only pair/s that can be used - they rely on a charuco board as the calibration target.

```
1) Stereo Calibration (charuco) - https://github.com/UoA-CARES/stereo_calibration

2) Marker Detection (aruco) - https://github.com/maraatech/aruco_detector
```

### Installing
Clone the package into the catkin directory you are using, presumed here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/UoA-CARES/cares_hand_eye_calibration.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

## Running
This package is intended to be called as part of the base platform bring up setup