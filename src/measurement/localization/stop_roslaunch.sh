#!/bin/bash
# Stop ros code locally

ps -aux | grep python | awk '{print $2}' | xargs kill -9
killall -q roslaunch roscore rosrun
pkill -f ros
