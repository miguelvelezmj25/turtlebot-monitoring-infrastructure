#!/usr/bin/python

import subprocess
import time
import os
import shutil
import re
import rospy
import socket
import ConfigParser
import monitors

config_parser = ConfigParser.RawConfigParser()
config_file_path = r'.serverconfig'
config_parser.read(config_file_path)
remote_host = config_parser.get(socket.gethostname(), 'simulator')
remote_password = config_parser.get(socket.gethostname(), 'password')

data_files = {}
gazebo_log_error = None
turtlebot_log_error = None

SUCCESS = 'success'
FAIL = 'fail'
RESULT = 'result'
NODE_NAME = 'turtlebot_remote'
NAVIGATION = 'map_navigation.py'
MONITOR_SCAN = 'scan_altered.py'
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
    time.sleep(7)

    subprocess.Popen("rosrun cp1_gazebo " + MONITOR_SCAN + ' "' + str(environment_configurations) + '"', shell=True,
                     stderr=turtlebot_log_error, stdout=turtlebot_log_error)
    time.sleep(7)

    # rospy.init_node(NODE_NAME, anonymous=True)
    subprocess.Popen("python " + MONITORS_FILE, shell=True)#, stderr=turtlebot_log_error,
                     # stdout=turtlebot_log_error)
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

    monitors.cleanup_files()
    all_monitors_file_name = monitors.DATA_FOLDER + monitors.MONITORS_FILE
    x_position_regex = '(?<=x: )(-)?[0-9]+.[0-9]+(e-[0-9]+)?'
    y_position_regex = '(?<=y: )(-)?[0-9]+.[0-9]+(e-[0-9]+)?'
    value_regex = '(?<=value: )(-)?[0-9]+.[0-9]+(e-[0-9]+)?'

    with open(all_monitors_file_name, 'r', 0) as all_monitors_file:
        all_monitors_filenames = all_monitors_file.read().splitlines(True)

        for monitor_file_name in all_monitors_filenames:
            monitor_name = monitor_file_name.split('.')[0]
            values = []
            with open(monitors.DATA_FOLDER + monitor_file_name.strip(), 'r', 0) as monitor_data_file:
                data = monitor_data_file.read().splitlines()

                for entry in data:
                    duration = re.search(time_regex, entry)
                    x_position = re.search(x_position_regex, entry)
                    y_position = re.search(y_position_regex, entry)
                    value = re.search(value_regex, entry)

                    # TODO Might need to change to float if we are going to get data all the time
                    data_point = [float(duration.group(0))]

                    if value is not None:
                        data_point.append(float(value.group(0)))

                        values.append(tuple(data_point))
                    elif x_position is not None and y_position is not None:
                        data_point.append(float(x_position.group(0)))
                        data_point.append(float(y_position.group(0)))

                        values.append(tuple(data_point))

            measurements[monitor_name] = values

    shutil.rmtree('data/' + str(id))

    # print measurements
    return measurements
