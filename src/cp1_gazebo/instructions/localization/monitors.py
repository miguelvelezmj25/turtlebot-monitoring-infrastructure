#!/usr/bin/python

import subprocess
import commands
import rospy
import os


from geometry_msgs.msg import PoseArray
from rosgraph_msgs.msg import Clock

DATA_FOLDER = 'monitor_data/'
MONITORS_FILE = 'monitors_file.txt'
CPU_MONITOR = 'cpu_monitor'
AMCL_CPU_MONITOR = 'amcl_cpu_monitor'

CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"

#
#

#
#
# def clock_proxy_for_cpu_callback(data, file):
#     data_time = rospy.get_rostime().secs
#
#     global cpu_current_time
#     if cpu_current_time == data_time:
#         return
#
#     cpu_current_time = data_time
#
#     value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]
#
#     if len(value) > 0:
#         file.write("time: {} | value: {}\n".format(cpu_current_time, float(value)))
#
#
# def particlecloud_proxy_for_amcl_cpu_callback(data, file):
#     data_time = rospy.get_rostime().secs
#
#     global amcl_cpu_current_time
#     if amcl_cpu_current_time == data_time:
#         return
#
#     amcl_cpu_current_time = data_time
#
#     value = commands.getstatusoutput(AMCL_CPU_MONITOR_COMMAND)[1]
#
#     if len(value) > 0:
#         file.write("time: {} | value: {}\n".format(amcl_cpu_current_time, float(value)))


def cleanup_files():
    all_monitors_file_name = DATA_FOLDER + MONITORS_FILE

    with open(all_monitors_file_name, 'r', 0) as all_monitors_file:
        data = all_monitors_file.read().splitlines()

        for data_file_name in data:
            with open(DATA_FOLDER + data_file_name, 'r', 0) as data_file:
                data = data_file.read().splitlines()

            with open(DATA_FOLDER + data_file_name, 'w', 0) as data_file:
                i = 1

                while i < len(data):
                    data_file.write(data[i] + '\n')
                    i += 1


if __name__ == '__main__':
    rospy.init_node('monitors_manager', anonymous=True)

    if not os.path.exists(DATA_FOLDER):
        os.mkdir(DATA_FOLDER)

    monitors = ['ground_truth_pose.py', 'estimate_pose.py']

    all_monitors_file = open(DATA_FOLDER + MONITORS_FILE, "w", 0)

    for monitor in monitors:
        all_monitors_file.write(monitor.split('.')[0] + ".txt\n")

    all_monitors_file.close()

    for monitor in monitors:
        subprocess.Popen('python ' + monitor, shell=True)


    #

    #
    # cpu_current_time = 0
    # monitor_file = open(DATA_FOLDER + CPU_MONITOR + '.txt', "w", 0)
    # all_monitors_file.write(CPU_MONITOR + ".txt\n")
    # rospy.Subscriber("/clock", Clock, callback=clock_proxy_for_cpu_callback,
    #                  callback_args=monitor_file, queue_size=1)
    #
    # amcl_cpu_current_time = 0
    # monitor_file = open(DATA_FOLDER + AMCL_CPU_MONITOR + '.txt', "w", 0)
    # all_monitors_file.write(AMCL_CPU_MONITOR + ".txt\n")
    # rospy.Subscriber("/particlecloud", PoseArray, callback=particlecloud_proxy_for_amcl_cpu_callback,
    #                  callback_args=monitor_file, queue_size=1)


    rospy.spin()


