#!/bin/bash

# Stop ros code in the simulator server
simulator=$1
password=$2

sshpass -p ${password} ssh -X ${simulator} ". ~/.bash_profile; $HOME/catkin_ws/src/cp1_gazebo/instructions/localization/stop_roslaunch.sh"
