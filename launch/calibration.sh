#!/bin/bash
#Launch the UR5 communication stuff
gnome-terminal -- bash -c "s2;roslaunch maara_ur5 ur5_real.launch; exec bash"
#Wait until comms with the UR5 is up and running
sleep 5
#Launch move group controller
gnome-terminal -- bash -c "s2;roslaunch ur5_moveit_config move_group.launch limited:=true; exec bash"
sleep 3
#Launch move group controller
gnome-terminal -- bash -c "s3;roslaunch rig_controller pair_controller.launch limited:=true; exec bash"
sleep 3
#Launch the calibration control code
gnome-terminal -- bash -c "s2;roslaunch maara_ur5 stereo_hand_eye_calibration.launch; exec bash"