#!/usr/bin/python

# For this script to work, you need to disable transform from odom to base_footprint published by turtlebot. We
# remap from tf to tf_altered in the gazebo node. This node publishes transform from odom to base_footprint using the
# new odom data.

import sys
import ast
import numpy
import rospy
from nav_msgs.msg import Odometry
import tf

gamma = 0
delta = 0
QUEUE_CAPACITY = 1000

broadcaster = tf.TransformBroadcaster()


def odom_callback(data):
    """
    odom_callback(data)

    This is the callback function for the /tf subscriber. It adds miscalibration and noise to the scan array based on
    the parameters specified by the user. It broadcasts the new odom data to tf.

    :param data: The message from the /odom topic
    """

    if delta > 0 and gamma > 0:
        # This is an interaction between two configurations
        translation = [numpy.random.normal(data.pose.pose.position.x * (1 + gamma), data.pose.pose.position.x * delta),
                       numpy.random.normal(data.pose.pose.position.y * (1 + gamma), data.pose.pose.position.y * delta)]
    elif delta > 0:
        try:
            translation = [numpy.random.normal(data.pose.pose.position.x, data.pose.pose.position.x * delta),
                           numpy.random.normal(data.pose.pose.position.y, data.pose.pose.position.y * delta)]
        except ValueError, ve:
            translation = [data.pose.pose.position.x, data.pose.pose.position.y]
    elif gamma != 0:
        translation = [data.pose.pose.position.x * (1 + gamma),
                       data.pose.pose.position.y * (1 + gamma)]
    else:
        translation = [data.pose.pose.position.x, data.pose.pose.position.y]

    translation.append(data.pose.pose.position.z)
    translation = tuple(translation)
    # TODO should we add noise and miscalibration to this?
    rotation = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                data.pose.pose.orientation.w)
    ros_time = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs)
    child = data.child_frame_id
    parent = data.header.frame_id

    broadcaster.sendTransform(translation, rotation, ros_time, child, parent)


if __name__ == '__main__':
    odometry_miscalibration = 'odometry_miscalibration'
    odometry_noise = 'odometry_noise'

    if len(sys.argv) == 1:
        args = "{}"
    else:
        args = sys.argv[-1]

    configurations = ast.literal_eval(args)

    if odometry_miscalibration in configurations:
        gamma = configurations[odometry_miscalibration]

    if odometry_noise in configurations:
        delta = configurations[odometry_noise]

    rospy.init_node('tf_altered')
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=QUEUE_CAPACITY)
    rospy.spin()
