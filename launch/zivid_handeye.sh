#ra_ur5!/bin/bash
#Launch the UR5 communication stuff
gnome-terminal --tab -- bash -c "s2;roslaunch uoa_ur_description ur5_real.launch limited:=true; exec bash"
#Wait until comms with the UR5 is up and running
sleep 5
#Launch move group controller
gnome-terminal --tab -- bash -c "s2;roslaunch ur5_moveit_config move_group.launch limited:=true; exec bash"
sleep 3
#Launch the mapping node
gnome-terminal --tab -- bash -c "s2;roslaunch cares_hand_eye_calibration zivid_handeye_calibration.launch; exec bash"
