#!/usr/bin/python

import commands
import rospy
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

DATA_FOLDER = 'monitor_data/'
GROUND_TRUTH_POSE = 'ground_truth_pose'
ESTIMATE_POSE = 'estimate_pose'
CPU_MONITOR = 'cpu_monitor'
AMCL_CPU_MONITOR = 'amcl_cpu_monitor'
MOVE_BASE_CPU_MONITOR = 'move_base_cpu_monitor'

gazebo_pose_data = []
amcl_pose_data = []
cpu_monitor_data = []
amcl_cpu_monitor_data = []
move_base_cpu_monitor_data = []

CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"
MOVE_BASE_CPU_MONITOR_COMMAND = "pidstat -t -C move_base 1 1 | grep -o .*-..move_base | sed 's/ *- *move_base//' | grep -o '......$'"


def gazebo_model_states_callback(data, arguments):
    print data
    print arguments
    data_time = rospy.get_rostime().secs

    global gazebo_current_time
    if gazebo_current_time == data_time:
        return

    gazebo_current_time = data_time
    pose = data.pose[2]
    position = pose.position

    gazebo_pose_data.append((gazebo_current_time, position.x, position.y))


def amcl_pose_callback(data):
    data_time = data.header.stamp.secs

    global amcl_current_time
    if amcl_current_time == data_time:
        return

    amcl_current_time = data_time
    pose = data.pose.pose
    position = pose.position

    amcl_pose_data.append((amcl_current_time, position.x, position.y))


def clock_proxy_for_cpu_callback(data):
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        cpu_monitor_data.append((cpu_current_time, float(value)))


def particlecloud_proxy_for_amcl_cpu_callback(data):
    data_time = rospy.get_rostime().secs

    global amcl_cpu_current_time
    if amcl_cpu_current_time == data_time:
        return

    amcl_cpu_current_time = data_time

    value = commands.getstatusoutput(AMCL_CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        amcl_cpu_monitor_data.append((amcl_cpu_current_time, float(value)))


def close_files():
    print "Closing monitor files"

    for monitor_files in monitor_files:
        monitor_files.close()

    print "Done closing monitor files"


rospy.on_shutdown(close_files())

if __name__ == '__main__':
    rospy.init_node('custom_monitors', anonymous=True)

    if not os.path.exists(DATA_FOLDER):
        os.mkdir(DATA_FOLDER)

    monitor_files = {}

    gazebo_current_time = 0
    monitor_files[GROUND_TRUTH_POSE] = open(DATA_FOLDER + GROUND_TRUTH_POSE + '.txt', "w", 0)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback=gazebo_model_states_callback,
                     callback_args=monitor_files[GROUND_TRUTH_POSE], queue_size=1)

    amcl_current_time = 0
    monitor_files[ESTIMATE_POSE] = open(DATA_FOLDER + ESTIMATE_POSE + '.txt', "w", 0)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=amcl_pose_callback,
                     callback_args=monitor_files[ESTIMATE_POSE], queue_size=1)

    cpu_current_time = 0
    monitor_files[CPU_MONITOR] = open(DATA_FOLDER + CPU_MONITOR + '.txt', "w", 0)
    rospy.Subscriber("/clock", Clock, callback=clock_proxy_for_cpu_callback,
                     callback_args=monitor_files[CPU_MONITOR], queue_size=1)

    amcl_cpu_current_time = 0
    monitor_files[AMCL_CPU_MONITOR] = open(DATA_FOLDER + AMCL_CPU_MONITOR + '.txt', "w", 0)
    rospy.Subscriber("/particlecloud", PoseArray, callback=particlecloud_proxy_for_amcl_cpu_callback,
                     callback_args=monitor_files[AMCL_CPU_MONITOR], queue_size=1)

    rospy.spin()
