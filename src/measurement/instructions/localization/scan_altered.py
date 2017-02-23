#!/usr/bin/python

# For this script to work, we remapped scan to scan_altered in the amcl node

import sys
import ast
import math
import numpy
import rospy
from sensor_msgs.msg import LaserScan

gamma = 0
delta = 0
array_size = -1
QUEUE_CAPACITY = 1000

publisher = rospy.Publisher('/scan_altered', LaserScan, queue_size=QUEUE_CAPACITY)


def scan_callback(data):
    """
    scan_callback(data)

    This is the callback function for the /scan subscriber. It adds miscalibration and noise to the scan array based on
    the parameters specified by the user. It publishes the new data to a new topic.

    :param data: The message from the /scan topic
    """
    new_values = []

    if delta > 0 and gamma > 0:
        # This is an interaction between two configurations
        for value in data.ranges:
            new_value = numpy.random.normal(value * (1 + gamma), value * delta)
            new_values.append(new_value)
    elif delta > 0:
        for value in data.ranges:
            new_value = numpy.random.normal(value, value * delta)
            new_values.append(new_value)
    elif gamma != 0:
        for value in data.ranges:
            new_values.append(value * (1 + gamma))

    if delta > 0 or gamma > 0:
        data.ranges = new_values

    if 0 <= array_size and array_size <= len(data.ranges):
        elements_to_remove = len(data.ranges) - array_size
        left_index = elements_to_remove >> 1
        right_index = len(data.ranges) - int(math.ceil(elements_to_remove / 2.0))
        data.ranges = data.ranges[left_index:right_index]

    publisher.publish(data)


if __name__ == '__main__':
    kinect_miscalibration = 'kinect_miscalibration'
    kinect_noise = 'kinect_noise'
    kinect_array = 'kinect_array'

    if len(sys.argv) == 1:
        args = "{}"
    else:
        args = sys.argv[-1]

    configurations = ast.literal_eval(args)

    if kinect_miscalibration in configurations:
        gamma = configurations[kinect_miscalibration]

    if kinect_noise in configurations:
        delta = configurations[kinect_noise]

    if kinect_array in configurations:
        array_size = configurations[kinect_array]

    rospy.init_node('scan_altered')
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=QUEUE_CAPACITY)
    rospy.spin()
