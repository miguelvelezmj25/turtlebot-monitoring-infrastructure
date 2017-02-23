#!/bin/bash
# Execute the experiment as many times as the user wants

iterations=$1

counter=1
while [ ${counter} -le ${iterations} ]
do
    python run_experiments.py
    ps -aux | grep python | awk '{print $2}' | xargs kill -9
    killall -q roslaunch roscore rosrun
    pkill -f ros
    counter=$(( $counter + 1 ))
    echo "Experiments left" $(( $iterations - $counter + 1 ))
done

# Since the setup.sh script added a line for ROS_MASTER_UR in .bash_profile,
# we need to remove that last line from the file
sed '$d' $HOME/.bash_profile > $HOME/.bash_profile-new
rm $HOME/.bash_profile
mv $HOME/.bash_profile-new $HOME/.bash_profile