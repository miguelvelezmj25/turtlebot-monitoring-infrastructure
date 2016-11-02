#!/bin/bash

# Stop ros code locally
killall -q roslaunch roscore rosrun
pkill -f ros
