#!/usr/bin/python

import sys
import ast
import numpy
import rospy
from sensor_msgs.msg import LaserScan

gamma = 0
delta = 0
queue_capacity = 1000

publisher = rospy.Publisher('/scan_altered', LaserScan, queue_size=queue_capacity)


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
    elif gamma > 0:
        for value in data.ranges:
            new_values.append(value * (1 + gamma))

        data.ranges = new_values

    publisher.publish(data)


if __name__ == '__main__':
    laser_miscalibration = 'laser_miscalibration'
    laser_noise = 'laser_noise'

    args = sys.argv[-1]

    if len(args) == 0:
        args = "{}"

    configurations = ast.literal_eval(args)

    if laser_miscalibration in configurations:
        gamma = configurations[laser_miscalibration]

    if laser_noise in configurations:
        delta = configurations[laser_noise]

    rospy.init_node('scan_monitor')
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=queue_capacity)
    rospy.spin()
