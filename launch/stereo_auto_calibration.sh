#!/bin/bash
#Launch the UR5 communication stuff
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash;roslaunch mini_rig_description ur5_real.launch; exec bash"
#Wait until comms with the UR5 is up and running
sleep 5
#Launch move group controller
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash;roslaunch ur5_moveit_config move_group.launch limited:=true; exec bash"
sleep 3
#Launch move group controller
gnome-terminal --tab -- bash -c "source ~/catkin_ws3/devel/setup.bash;roslaunch stereo_inference_ros stereo_pair.launch cameras:=stereo_pair_uoa method:=Rectify; exec bash"
sleep 3
#Launch the calibration control code
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash;roslaunch cares_hand_eye_calibration stereo_auto_calibration.launch; exec bash"
