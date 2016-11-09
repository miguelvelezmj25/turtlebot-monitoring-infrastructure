#!/usr/bin/python

import rospy
import monitors
import commands

from rosgraph_msgs.msg import Clock

CPU_MONITOR = 'cpu_monitor'
CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"


def clock_proxy_for_cpu_callback(data, file):
    # data_time = rospy.get_rostime().to_sec()
    data_time = rospy.get_rostime().secs

    global cpu_current_time
    if cpu_current_time == data_time:
        return

    cpu_current_time = data_time

    value = commands.getstatusoutput(CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        file.write("time: {} | value: {}\n".format(cpu_current_time, float(value)))


if __name__ == '__main__':
    rospy.init_node('ground_truth_pose_monitor', anonymous=True)

    cpu_current_time = 0
    monitor_file = open(monitors.DATA_FOLDER + CPU_MONITOR + '.txt', "w", 0)
    rospy.Subscriber("/clock", Clock, callback=clock_proxy_for_cpu_callback, callback_args=monitor_file, queue_size=1)
    rospy.spin()
