#!/usr/bin/python

import sys
import ast
import numpy
import rospy
from sensor_msgs.msg import LaserScan

gamma = 0
delta = 0
laser_miscalibration = 'laser_miscalibration'
laser_noise = 'laser_noise'
queue_capacity = 1000
publisher = rospy.Publisher('/scan_altered', LaserScan, queue_size=queue_capacity)


def scan_callback(data):
    """

    :param data:
    :return:
    """
    new_values = []

    if delta > 0 and gamma > 0:
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
