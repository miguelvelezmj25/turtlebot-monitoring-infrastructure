#!/bin/bash

iterations=$1

counter=1
while [ ${counter} -le ${iterations} ]
do
    python run_experiments.py
    ps -aux | grep python | awk '{print $2}' | xargs kill -9
    killall -q roslaunch roscore
    pkill -f ros
    counter=$(( $counter + 1 ))
done

sed '$d' $HOME/.bash_profile > $HOME/.bash_profile-new
rm $HOME/.bash_profile
mv $HOME/.bash_profile-new $HOME/.bash_profile