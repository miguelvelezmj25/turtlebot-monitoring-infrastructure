#!/bin/bash

# Start gazebo in the server acting as simulator
simulator=$1
password=$2

sshpass -p ${password} ssh -X ${simulator} ". ~/.bash_profile; $HOME/catkin_ws/src/cp1_gazebo/instructions/localization/start_gazebo.sh"
