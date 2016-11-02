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

gazebo_pose_data = []
amcl_pose_data = []
cpu_monitor_data = []
amcl_cpu_monitor_data = []
move_base_cpu_monitor_data = []
gazebo_log_error = None
turtlebot_log_error = None

config_parser = ConfigParser.RawConfigParser()
config_file_path = r'.serverconfig'
config_parser.read(config_file_path)
remote_host = config_parser.get(socket.gethostname(), 'simulator')
remote_password = config_parser.get(socket.gethostname(), 'password')


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

    subprocess.Popen("rosrun cp1_gazebo " + monitor_scan + '.py "' + str(environment_configurations) + '"', shell=True,
                     stderr=turtlebot_log_error, stdout=turtlebot_log_error)
    time.sleep(10)

    rospy.init_node(node_name, anonymous=True)
    global gazebo_current_time
    gazebo_current_time = 0
    global amcl_current_time
    amcl_current_time = 0
    global cpu_current_time
    cpu_current_time = 0
    global amcl_cpu_current_time
    amcl_cpu_current_time = 0
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_callback, queue_size=1)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.Subscriber("/clock", Clock, cpu_callback, queue_size=1)
    rospy.Subscriber("/particlecloud", PoseArray, amcl_callback, queue_size=1)


def gazebo_callback(data):
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


def cpu_callback(data):
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(cpu_monitor_command)[1]

    if len(value) > 0:
        global cpu_monitor_data
        cpu_monitor_data.append((cpu_current_time, float(value)))


def amcl_callback(data):
    data_time = rospy.get_rostime().secs

    global amcl_cpu_current_time
    if amcl_cpu_current_time == data_time:
        return

    amcl_cpu_current_time = data_time

    value = commands.getstatusoutput(amcl_cpu_monitor_command)[1]

    if len(value) > 0:
        global amcl_cpu_monitor_data
        amcl_cpu_monitor_data.append((amcl_cpu_current_time, float(value)))


def shutdown():
    print "Shutdown"

    subprocess.Popen("killall -q roslaunch roscore rosrun", shell=True)
    subprocess.Popen("pkill -f ros", shell=True)

    if not os.path.exists('log/'):
        os.mkdir('log/')
    global gazebo_log_error
    gazebo_log_error = open('log/gazebo_remote.log', 'a+', 0)

    subprocess.Popen("./stop_remote_roslaunch.sh " + str(remote_host) + ' ' + str(remote_password), shell=True, stdout=gazebo_log_error, stderr=gazebo_log_error)
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
    data_files[navigation] = open('data/' + str(id) + "/" + navigation + '.txt', 'a+', 0)
    subprocess.call("python " + navigation + '.py "' + str(configurations) + '"', shell=True,
                    stdout=data_files[navigation])

    time.sleep(2)
    time_regex = '(?<=time: )[0-9]+.[0-9]+'
    fail_regex = 'fail'

    with open(data_files[navigation].name, 'r', 0) as content_file:
        content = content_file.read()

    duration = re.search(time_regex, content)
    fail = re.search(fail_regex, content)

    measurements = {}
    if fail is not None:
        duration = None
        measurements['result'] = 'fail'

    else:
        duration = duration.group(0)
        measurements['result'] = 'success'

    measurements['duration'] = duration
    measurements['ground_truth_pose'] = gazebo_pose_data
    measurements['estimate_pose'] = amcl_pose_data
    measurements['cpu_monitor'] = cpu_monitor_data
    measurements['amcl_monitor'] = amcl_cpu_monitor_data
    measurements['move_base_monitor'] = move_base_cpu_monitor_data

    shutil.rmtree('data/' + str(id))

    return measurements


data_files = {}
node_name = 'turtlebot_remote'
navigation = 'map_navigation'
monitor_scan = 'monitor_scan'
cpu_monitor_command = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
amcl_cpu_monitor_command = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"
move_base_cpu_monitor_command = "pidstat -t -C move_base 1 1 | grep -o .*-..move_base | sed 's/ *- *move_base//' | grep -o '......$'"

