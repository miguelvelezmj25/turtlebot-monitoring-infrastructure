#!/usr/bin/python

import sys
import ast
import numpy
import rospy
from sensor_msgs.msg import LaserScan

gamma = 0
delta = 0
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

        data.ranges = new_values
    elif delta > 0:
        for value in data.ranges:
            new_value = numpy.random.normal(value, value * delta)
            new_values.append(new_value)

        data.ranges = new_values
    elif gamma != 0:
        for value in data.ranges:
            new_values.append(value * (1 + gamma))

        data.ranges = new_values

    publisher.publish(data)


if __name__ == '__main__':
    kinect_miscalibration = 'kinect_miscalibration'
    kinect_noise = 'kinect_noise'

    if len(sys.argv) == 1:
        args = "{}"
    else:
        args = sys.argv[-1]

    configurations = ast.literal_eval(args)

    if kinect_miscalibration in configurations:
        gamma = configurations[kinect_miscalibration]

    if kinect_noise in configurations:
        delta = configurations[kinect_noise]

    rospy.init_node('scan_monitor')
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=QUEUE_CAPACITY)
    rospy.spin()
