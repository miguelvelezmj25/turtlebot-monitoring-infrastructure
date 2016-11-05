#!/usr/bin/python

import commands
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

gazebo_pose_data = []
amcl_pose_data = []
cpu_monitor_data = []
amcl_cpu_monitor_data = []
move_base_cpu_monitor_data = []

CPU_MONITOR_COMMAND = "mpstat 1 1 | grep -o M..all........ | sed -e 's/M  all   //'"
AMCL_CPU_MONITOR_COMMAND = "pidstat -t -C amcl 1 1 | grep -o .*-..amcl | sed 's/ *- *amcl//' | grep -o '......$'"
MOVE_BASE_CPU_MONITOR_COMMAND = "pidstat -t -C move_base 1 1 | grep -o .*-..move_base | sed 's/ *- *move_base//' | grep -o '......$'"


def gazebo_model_states_callback(data):
    data_time = rospy.get_rostime().secs

    global gazebo_current_time
    if gazebo_current_time == data_time:
        return

    gazebo_current_time = data_time
    pose = data.pose[2]
    position = pose.position

    gazebo_pose_data.append((gazebo_current_time, position.x, position.y))
    print gazebo_pose_data


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


if __name__ == '__main__':
    rospy.init_node('custom_monitors', anonymous=True)

    global gazebo_current_time
    gazebo_current_time = 0
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_states_callback, queue_size=1)

    global amcl_current_time
    amcl_current_time = 0
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)

    global cpu_current_time
    cpu_current_time = 0
    rospy.Subscriber("/clock", Clock, clock_proxy_for_cpu_callback, queue_size=1)

    global amcl_cpu_current_time
    amcl_cpu_current_time = 0
    rospy.Subscriber("/particlecloud", PoseArray, particlecloud_proxy_for_amcl_cpu_callback, queue_size=1)

    rospy.spin()
