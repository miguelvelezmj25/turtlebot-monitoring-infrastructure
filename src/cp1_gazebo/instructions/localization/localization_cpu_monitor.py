#!/usr/bin/python

import rospy
import monitors
import commands

from geometry_msgs.msg import PoseArray

LOCALIZATION_CPU_MONITOR = 'localization_cpu_monitor'
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"


def particlecloud_proxy_for_amcl_cpu_callback(data, file):
    # data_time = rospy.get_rostime().to_sec()
    data_time = rospy.get_rostime().secs

    global amcl_cpu_current_time
    if amcl_cpu_current_time == data_time:
        return

    amcl_cpu_current_time = data_time

    value = commands.getstatusoutput(AMCL_CPU_MONITOR_COMMAND)[1]

    if len(value) > 0:
        file.write("time: {} | value: {}\n".format(amcl_cpu_current_time, float(value)))


if __name__ == '__main__':
    rospy.init_node('ground_truth_pose_monitor', anonymous=True)

    amcl_cpu_current_time = 0
    monitor_file = open(monitors.DATA_FOLDER + LOCALIZATION_CPU_MONITOR + '.txt', "w", 0)
    rospy.Subscriber("/particlecloud", PoseArray, callback=particlecloud_proxy_for_amcl_cpu_callback,
                     callback_args=monitor_file, queue_size=1)
    rospy.spin()
