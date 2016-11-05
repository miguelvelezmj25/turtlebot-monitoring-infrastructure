#!/usr/bin/python

import subprocess
import time
import os
import shutil
import re
import rospy
import socket
import ConfigParser

config_parser = ConfigParser.RawConfigParser()
config_file_path = r'.serverconfig'
config_parser.read(config_file_path)
remote_host = config_parser.get(socket.gethostname(), 'simulator')
remote_password = config_parser.get(socket.gethostname(), 'password')

gazebo_pose_data = []
amcl_pose_data = []
cpu_monitor_data = []
amcl_cpu_monitor_data = []
move_base_cpu_monitor_data = []
data_files = {}

gazebo_log_error = None
turtlebot_log_error = None

SUCCESS = 'success'
FAIL = 'fail'
RESULT = 'result'
NODE_NAME = 'turtlebot_remote'
NAVIGATION = 'map_navigation.py'
MONITOR_SCAN = 'monitor_scan.py'
MONITORS_FILE = 'monitors.py'
DURATION = 'duration'
GROUND_TRUTH_POSE = 'ground_truth_pose'
ESTIMATE_POSE = 'estimate_pose'
CPU_MONITOR = 'cpu_monitor'
AMCL_CPU_MONITOR = 'amcl_cpu_monitor'
MOVE_BASE_CPU_MONITOR = 'move_base_cpu_monitor'


def startup(environment_configurations):
    print "Startup"

    global gazebo_log_error
    gazebo_log_error = open('log/gazebo_remote.log', 'a+', 0)
    subprocess.Popen("./start_remote_gazebo.sh " + str(remote_host) + ' ' + str(remote_password), shell=True,
                     stderr=gazebo_log_error, stdout=gazebo_log_error)
    time.sleep(10)

    global turtlebot_log_error
    turtlebot_log_error = open('log/turtlebot_remote.log', 'a+', 0)
    subprocess.Popen("roslaunch cp1_gazebo robot-altered.launch", shell=True, stderr=turtlebot_log_error,
                     stdout=turtlebot_log_error)
    time.sleep(5)

    subprocess.Popen("rosrun cp1_gazebo " + MONITOR_SCAN + ' "' + str(environment_configurations) + '"', shell=True,
                     stderr=turtlebot_log_error, stdout=turtlebot_log_error)
    time.sleep(10)

    rospy.init_node(NODE_NAME, anonymous=True)
    subprocess.Popen("rosrun cp1_gazebo " + MONITORS_FILE, shell=True, stderr=turtlebot_log_error,
                     stdout=turtlebot_log_error)
    time.sleep(5)


def shutdown():
    print "Shutdown"

    subprocess.Popen("killall -q roslaunch roscore rosrun", shell=True)
    subprocess.Popen("pkill -f ros", shell=True)

    if not os.path.exists('log/'):
        os.mkdir('log/')
    global gazebo_log_error
    gazebo_log_error = open('log/gazebo_remote.log', 'a+', 0)

    subprocess.Popen("./stop_remote_roslaunch.sh " + str(remote_host) + ' ' + str(remote_password), shell=True,
                     stdout=gazebo_log_error, stderr=gazebo_log_error)
    time.sleep(20)

    gazebo_log_error.close()

    if turtlebot_log_error is not None:
        turtlebot_log_error.close()


def restart(environment_configurations):
    print "Restart"
    shutdown()
    startup(environment_configurations)


def measure(id, configurations):
    print "Measuring for " + str(id)

    if os.path.exists('data/' + str(id)):
        shutil.rmtree('data/' + str(id))
    os.makedirs('data/' + str(id))
    data_files[NAVIGATION] = open('data/' + str(id) + "/" + NAVIGATION + '.txt', 'a+', 0)
    subprocess.call("python " + NAVIGATION + ' "' + str(configurations) + '"', shell=True,
                    stdout=data_files[NAVIGATION])

    time.sleep(2)
    time_regex = '(?<=time: )[0-9]+.[0-9]+'
    fail_regex = 'fail'

    with open(data_files[NAVIGATION].name, 'r', 0) as content_file:
        content = content_file.read()

    duration = re.search(time_regex, content)
    fail = re.search(fail_regex, content)

    measurements = {}
    if fail is not None:
        duration = None
        measurements[RESULT] = FAIL

    else:
        duration = duration.group(0)
        measurements[RESULT] = SUCCESS

    measurements[DURATION] = duration
    measurements[GROUND_TRUTH_POSE] = gazebo_pose_data
    measurements[ESTIMATE_POSE] = amcl_pose_data
    measurements[CPU_MONITOR] = cpu_monitor_data
    measurements[AMCL_CPU_MONITOR] = amcl_cpu_monitor_data
    measurements[MOVE_BASE_CPU_MONITOR] = move_base_cpu_monitor_data

    shutil.rmtree('data/' + str(id))

    return measurements
