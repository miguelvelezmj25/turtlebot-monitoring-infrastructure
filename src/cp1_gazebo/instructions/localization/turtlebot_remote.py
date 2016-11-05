#!/usr/bin/python

import subprocess
import time
import os
import shutil
import re
import rospy
import commands
import socket
import ConfigParser

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

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

GAZEBO_LOG_ERROR = None
TURTLEBOT_LOG_ERROR = None

SUCCESS = 'success'
FAIL = 'fail'
RESULT = 'result'
NODE_NAME = 'turtlebot_remote'
NAVIGATION = 'map_navigation'
MONITOR_SCAN = 'monitor_scan'
DURATION = 'duration'
GROUND_TRUTH_POSE = 'ground_truth_pose'
ESTIMATE_POSE = 'estimate_pose'
CPU_MONITOR = 'cpu_monitor'
AMCL_CPU_MONITOR = 'amcl_cpu_monitor'
MOVE_BASE_CPU_MONITOR = 'move_base_cpu_monitor'
CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"
MOVE_BASE_CPU_MONITOR_COMMAND = "pidstat -t -C move_base 1 1 | grep -o .*-..move_base | sed 's/ *- *move_base//' | grep -o '......$'"


def startup(environment_configurations):
    print "Startup"

    global GAZEBO_LOG_ERROR
    GAZEBO_LOG_ERROR = open('log/gazebo_remote.log', 'a+', 0)
    subprocess.Popen("./start_remote_gazebo.sh " + str(remote_host) + ' ' + str(remote_password), shell=True,
                     stderr=GAZEBO_LOG_ERROR, stdout=GAZEBO_LOG_ERROR)
    time.sleep(10)

    global TURTLEBOT_LOG_ERROR
    TURTLEBOT_LOG_ERROR = open('log/turtlebot_remote.log', 'a+', 0)
    subprocess.Popen("roslaunch cp1_gazebo robot-altered.launch", shell=True, stderr=TURTLEBOT_LOG_ERROR,
                     stdout=TURTLEBOT_LOG_ERROR)
    time.sleep(5)

    subprocess.Popen("rosrun cp1_gazebo " + MONITOR_SCAN + '.py "' + str(environment_configurations) + '"', shell=True,
                     stderr=TURTLEBOT_LOG_ERROR, stdout=TURTLEBOT_LOG_ERROR)
    time.sleep(10)

    rospy.init_node(NODE_NAME, anonymous=True)
    global gazebo_current_time
    gazebo_current_time = 0
    global amcl_current_time
    amcl_current_time = 0
    global cpu_current_time
    cpu_current_time = 0
    global amcl_cpu_current_time
    amcl_cpu_current_time = 0
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_states_callback, queue_size=1)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.Subscriber("/clock", Clock, clock_proxy_for_cpu_callback, queue_size=1)
    rospy.Subscriber("/particlecloud", PoseArray, particlecloud_proxy_for_amcl_cpu_callback, queue_size=1)


def gazebo_model_states_callback(data):
    data_time = rospy.get_rostime().secs

    global gazebo_current_time
    if gazebo_current_time == data_time:
        return

    gazebo_current_time = data_time
    pose = data.pose[2]
    position = pose.position

    global gazebo_pose_data
    gazebo_pose_data.append((gazebo_current_time, position.x, position.y))


def amcl_pose_callback(data):
    data_time = data.header.stamp.secs

    global amcl_current_time
    if amcl_current_time == data_time:
        return

    amcl_current_time = data_time
    pose = data.pose.pose
    position = pose.position

    global amcl_pose_data
    amcl_pose_data.append((amcl_current_time, position.x, position.y))


def clock_proxy_for_cpu_callback(data):
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        global cpu_monitor_data
        cpu_monitor_data.append((cpu_current_time, float(value)))


def particlecloud_proxy_for_amcl_cpu_callback(data):
    data_time = rospy.get_rostime().secs

    global amcl_cpu_current_time
    if amcl_cpu_current_time == data_time:
        return

    amcl_cpu_current_time = data_time

    value = commands.getstatusoutput(AMCL_CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        global amcl_cpu_monitor_data
        amcl_cpu_monitor_data.append((amcl_cpu_current_time, float(value)))


def shutdown():
    print "Shutdown"

    subprocess.Popen("killall -q roslaunch roscore rosrun", shell=True)
    subprocess.Popen("pkill -f ros", shell=True)

    if not os.path.exists('log/'):
        os.mkdir('log/')
    global GAZEBO_LOG_ERROR
    GAZEBO_LOG_ERROR = open('log/gazebo_remote.log', 'a+', 0)

    subprocess.Popen("./stop_remote_roslaunch.sh " + str(remote_host) + ' ' + str(remote_password), shell=True,
                     stdout=GAZEBO_LOG_ERROR, stderr=GAZEBO_LOG_ERROR)
    time.sleep(20)

    GAZEBO_LOG_ERROR.close()

    if TURTLEBOT_LOG_ERROR is not None:
        TURTLEBOT_LOG_ERROR.close()


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
    subprocess.call("python " + NAVIGATION + '.py "' + str(configurations) + '"', shell=True,
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
