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
	a) sudo apt-get install ros-noetic-vision-visp
```

### Calabration and Marker Services
Hand eye calibration relies on other packages/services to conduct the stereo calibration and marker detection.\
These can be replaced with different vairations by changing the respective service topics in the launch files.\
The two below are the ones specifically designed for this package but are not the only pair/s that can be used - they rely on a charuco board as the calibration target.

```
1) Stereo Calibration (charuco) - https://github.com/UoA-CARES/stereo_calibration

2) Marker Detection (aruco) - https://github.com/maraatech/aruco_detector
```

NOTE: Stereo Calibration is only required if using stereo cameras - depth cameras this can be omitted.

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
This package is intended to be called as part of a robot setup - below are the base launch files you can call from your robot package setup. The launch files stereo_auto_calibration.launch and depth_auto_calibration.launch can be included in your primary robot calibration launch file stored in your robots bringup package - depending on if it is a stereo camera or depth sensor. Do NOT modify these launch files as part of your setup! Include them within your own roobt's calibration launch file and pass through the configurations you require! Consider the launch files like a method call from the given robot's package.

### Stereo Hand-Eye Calibration - stereo_auto_calibration.launch
This launch file can be included for calibrating a pair of stereo cameras and then the hand-eye calibration.

```xml
<?xml version="1.0"?>
<launch>
    <!-- Default blank to do live calibration -->
    <arg name="file_path" default=""/>

    <!-- Init pose of the arm -->
    <arg name="init_x"  default="0.0"/>
    <arg name="init_y"  default="0.65"/>
    <arg name="init_z"  default="0.35"/>

    <arg name="planning_link" default="base_link"/>

    <arg name="platform_server" default="server"/>

    <arg name="sensor" default=""/>
    <!-- Stereo camera topics -->
    <!-- NOTE: precalibration we only have left and right images to work with -->
    <arg name="sensor_topics" default="$(find cares_hand_eye_calibration)/config/stereo_calibration.yaml"/>

    <arg name="display" default="true"/>

    <!-- The input reference frames -->
    <arg name="robot_base_frame"     default="base_link" />
    <arg name="robot_effector_frame" default="stereo_pair/base_link" />
    <arg name="tracking_base_frame"  default="stereo_pair/left_frame" />

    <!-- Which control frame will be used to move the sensor around during calibration -->
    <arg name="robot_control_frame"  default="$(arg robot_effector_frame)" />

    <!--Charuco Board Parameters-->
    <arg name="board_width"   default="8"/><!--Number of corners-->
    <arg name="board_height"  default="5"/><!--Number of corners-->

    <arg name="square_length" default="45"/><!--mm-->
    <arg name="marker_length" default="23.0"/><!--mm-->

    <arg name="image_width"  default="1936"/>
    <arg name="image_height" default="1216"/>
    
    <!--1 Charuco-->
    <arg name="dictionary" default="3"/>
    <arg name="method"     default="1"/>

    <!-- Aruco Diamond IDs -->
    <arg name="centre"    default="11"/>
    <arg name="top_left"  default="6"/>
    <arg name="top_right" default="7"/>
    <arg name="bot_right" default="16"/>
    <arg name="bot_left"  default="15"/>

    <include file="$(find cares_hand_eye_calibration)/launch/auto_calibration.launch" pass_all_args="true"> 
    </include>
    
    <!-- Stereo Diamond Detector Service -->     
    <include file="$(find aruco_detector)/launch/stereo_diamond_detector_service.launch" pass_all_args="true">
    </include>

    <!-- Stereo Calibration Service -->
    <include file="$(find stereo_calibration)/launch/service_stereo_calibration.launch" pass_all_args="true">
    </include>    
</launch>

```

### Depth Camera Hand-Eye Calibrtion - depth_auto_calibration.launch
This launch file can be included for hand-eye calibration of a depth sensor.

```xml
<?xml version="1.0"?>
<launch>
    <!-- Default blank to do live calibration -->
    <arg name="file_path" default=""/>

    <!-- Init pose of the arm -->
    <arg name="init_x"  default="0.0"/>
    <arg name="init_y"  default="0.65"/>
    <arg name="init_z"  default="0.35"/>

    <arg name="planning_link" default="base_link"/>

    <arg name="platform_server" default="server"/>

    <arg name="sensor" default=""/>
    <!-- NOTE: as the depth camera is precalibrated we can jump straight to depth data -->
    <arg name="sensor_topics" default="$(find cares_hand_eye_calibration)/config/depth.yaml"/>

    <arg name="display" default="true"/>

    <!-- The input reference frames -->
    <arg name="robot_base_frame"     default="base_link" />
    <arg name="robot_effector_frame" default="stereo_pair/base_link" />
    <arg name="tracking_base_frame"  default="stereo_pair/left_frame" />

    <!-- Which control frame will be used to move the sensor around during calibration -->
    <arg name="robot_control_frame"  default="$(arg robot_effector_frame)" />

    <!-- Aruco Diamond IDs -->
    <arg name="dictionary" default="3"/>
    <arg name="centre"     default="10"/>
    <!-- <arg name="top_left"   default="7"/>
    <arg name="top_right"  default="8"/>
    <arg name="bot_right"  default="14"/>
    <arg name="bot_left"   default="13"/> -->

    <arg name="is_depth_in_meters" default="true" />

    <include file="$(find cares_hand_eye_calibration)/launch/auto_calibration.launch" pass_all_args="true"> 
    </include> 

    <include file="$(find aruco_detector)/launch/depth_detector_service.launch" pass_all_args="true">
    </include>

    <!-- Depth Diamond Detector Service -->     
    <!-- <include file="$(find aruco_detector)/launch/depth_diamond_detector_service.launch" pass_all_args="true">
    </include> -->
</launch>

```

### How to setup on your base platform
An example of this with the "your_robot" platform using a stereo pair is shown below which would be placed within "your_robot_bringup" package. You will also need to run "your_robot" control - follow the instructions in the given robot's README file to start the robot's control.

```xml
<?xml version="1.0" ?>
<launch>
	<arg name="robot_base_frame"      default="base_link" />
	<arg name="robot_effector_frame"  default="your_robot/base_link" />
	<arg name="tracking_base_frame"   default="your_robot_sensor/your_robot_sensor_frame"/>

	<arg name="sensor_topics" default="$(find your_robot_bringup)/config/sensors_calibration_topics.yaml"/>

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