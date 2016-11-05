#!/usr/bin/python

import commands
import rospy
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

DATA_FOLDER = 'monitor_data/'
MONITORS_FILE = 'monitors_file.txt'
GROUND_TRUTH_POSE = 'ground_truth_pose'
ESTIMATE_POSE = 'estimate_pose'
CPU_MONITOR = 'cpu_monitor'
AMCL_CPU_MONITOR = 'amcl_cpu_monitor'
MOVE_BASE_CPU_MONITOR = 'move_base_cpu_monitor'

CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"
MOVE_BASE_CPU_MONITOR_COMMAND = "pidstat -t -C move_base 1 1 | grep -o .*-..move_base | sed 's/ *- *move_base//' | grep -o '......$'"


def gazebo_model_states_callback(data, file):
    data_time = rospy.get_rostime().secs

    global gazebo_current_time
    if gazebo_current_time == data_time:
        return

    gazebo_current_time = data_time
    pose = data.pose[2]
    position = pose.position

    file.write("time: {} | x: {} | y: {}\n".format(gazebo_current_time, position.x, position.y))


def amcl_pose_callback(data, file):
    data_time = data.header.stamp.secs

    global amcl_current_time
    if amcl_current_time == data_time:
        return

    amcl_current_time = data_time
    pose = data.pose.pose
    position = pose.position

    file.write("time: {} | x: {} | y: {}\n".format(amcl_current_time, position.x, position.y))


def clock_proxy_for_cpu_callback(data, file):
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        file.write("time: {} | value: {}\n".format(cpu_current_time, float(value)))


def particlecloud_proxy_for_amcl_cpu_callback(data, file):
    data_time = rospy.get_rostime().secs

    global amcl_cpu_current_time
    if amcl_cpu_current_time == data_time:
        return

    amcl_cpu_current_time = data_time

    value = commands.getstatusoutput(AMCL_CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        file.write("time: {} | value: {}\n".format(amcl_cpu_current_time, float(value)))


def cleanup_files():
    all_monitors_file_name = DATA_FOLDER + MONITORS_FILE + '.txt'

    with open(all_monitors_file_name, 'r', 0) as all_monitors_file:
        data = all_monitors_file.read().splitlines(True)

    with open(all_monitors_file_name, 'r', 0) as all_monitors_file:
        all_monitors_file.writelines(data[1:])


if __name__ == '__main__':
    rospy.init_node('custom_monitors', anonymous=True)

    if not os.path.exists(DATA_FOLDER):
        os.mkdir(DATA_FOLDER)

    all_monitors_file = open(DATA_FOLDER + MONITORS_FILE + '.txt', "w", 0)

    gazebo_current_time = 0
    monitor_file = open(DATA_FOLDER + GROUND_TRUTH_POSE + '.txt', "w", 0)
    all_monitors_file.write(GROUND_TRUTH_POSE + ".txt\n")
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback=gazebo_model_states_callback,
                     callback_args=monitor_file, queue_size=1)

    amcl_current_time = 0
    monitor_file = open(DATA_FOLDER + ESTIMATE_POSE + '.txt', "w", 0)
    all_monitors_file.write(ESTIMATE_POSE + ".txt\n")
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=amcl_pose_callback,
                     callback_args=monitor_file, queue_size=1)

    cpu_current_time = 0
    monitor_file = open(DATA_FOLDER + CPU_MONITOR + '.txt', "w", 0)
    all_monitors_file.write(CPU_MONITOR + ".txt\n")
    rospy.Subscriber("/clock", Clock, callback=clock_proxy_for_cpu_callback,
                     callback_args=monitor_file, queue_size=1)

    amcl_cpu_current_time = 0
    monitor_file = open(DATA_FOLDER + AMCL_CPU_MONITOR + '.txt', "w", 0)
    all_monitors_file.write(AMCL_CPU_MONITOR + ".txt\n")
    rospy.Subscriber("/particlecloud", PoseArray, callback=particlecloud_proxy_for_amcl_cpu_callback,
                     callback_args=monitor_file, queue_size=1)

    all_monitors_file.close()
    rospy.spin()


