#!/usr/bin/python

import commands

import rospy
from rosgraph_msgs.msg import Clock

import monitors

CPU_MONITOR = 'cpu_monitor'
# mpstat gives the CPU consumption based on the entire CPU. We need to multiply it by the number of cores to get a
# measurement per CPU.
CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
CORES_NUMBER = 4


def clock_proxy_for_cpu_callback(data, file):
    # data_time = rospy.get_rostime().to_sec()
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        file.write("time: {} | value: {}\n".format(cpu_current_time, float(value) * CORES_NUMBER))


if __name__ == '__main__':
    rospy.init_node(CPU_MONITOR, anonymous=True)

    cpu_current_time = 0
    monitor_file = open(monitors.DATA_FOLDER + CPU_MONITOR + '.txt', "w", 0)
    rospy.Subscriber("/clock", Clock, callback=clock_proxy_for_cpu_callback, callback_args=monitor_file, queue_size=1)
    rospy.spin()
