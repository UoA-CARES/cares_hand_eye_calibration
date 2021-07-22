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

### How to setup on your base platform
The launch files stereo_auto_calibration.launch and depth_auto_calibration.launch can be included in your primary robot calibration launch file stored in your robots bringup package - depending on if it is a stereo camera or depth sensor. An example of this with the "your_robot" platform using a stereo pair is shown below which would be placed within "your_robot_bringup" package.

You will also need to run "your_robot" control - follow the instructions in the given robot's README file to start the robot's control.

```xml
<?xml version="1.0" ?>
<launch>
	<arg name="robot_base_frame"      default="base_link" />
	<arg name="robot_effector_frame"  default="your_robot_sensor/base_link" />
	<arg name="tracking_base_frame"   default="your_robot_sensor/your_robot_sensor_frame"/>

	<arg name="sensor_topics" default="$(find your_robot_bringup)/config/sensors_calibration.yaml"/>

	<!--Charuco Board Parameters-->
	<arg name="board_width"   default="8"/><!--Number of corners-->
	<arg name="board_height"  default="5"/><!--Number of corners-->

	<arg name="square_length" default="45"/><!--mm-->
	<arg name="marker_length" default="23.0"/><!--mm--> 

	<arg name="image_width"  default="2464"/>
	<arg name="image_height" default="2056"/>

	<!-- Diamond Marker IDs -->
	<arg name="centre"    default="11"/>
	<arg name="top_left"  default="6"/>
	<arg name="top_right" default="7"/>
	<arg name="bot_right" default="16"/>
	<arg name="bot_left"  default="15"/>

	<include file="$(find cares_hand_eye_calibration)/launch/stereo_auto_calibration.launch" pass_all_args="true">
	 
	</include>
</launch>

```

## Sensor Topics
Sensor topics 