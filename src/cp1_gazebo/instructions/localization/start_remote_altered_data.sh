#!/bin/bash
# Start the altered data scripts in the server acting as simulator. Be aware that this would not start the scripts if you are using the same
# machine as host

simulator=$1
password=$2
file=$3
configuration=$4

sshpass -p ${password} ssh -X ${simulator} ". ~/.bash_profile; $HOME/catkin_ws/src/cp1_gazebo/instructions/localization/start_altered_data.sh "${file} ${configuration}
