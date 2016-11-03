#!/bin/bash
# Determine what server should act as the simulator for this job

input=".serverconfig"
found=false

while IFS= read -r var
do
    if [ "[$HOSTNAME]" = "$var" ]
    then
        read -r var
        IFS=' = ' read -ra simulator <<< "$var"
        echo 'export ROS_MASTER_URI=http://'${simulator[1]}':11311' >> $HOME/.bash_profile
    fi
done <"$input"