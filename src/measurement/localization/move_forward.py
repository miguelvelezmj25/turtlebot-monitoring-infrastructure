#!/usr/bin/python
import roslib;

roslib.load_manifest('ig_action_msgs')
import rospy

from gazebo_msgs.srv import GetModelState

import actionlib
import ig_action_msgs.msg
import sys
import threading
import tf.transformations as tf
import requests
import math
import argparse

# These are translation coordinates between the map and gazebo
# The two things should really be syncrhonized
X_MAP_TO_GAZEBO_TRANSLATION = 56
Y_MAP_TO_GAZEBO_TRANSLATION = 42

MAX_RUN_TIME = 80


def getTurtleBotState():
    try:
        getModelStateSrv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        tb = getModelStateSrv('mobile_base', '')
        q = (tb.pose.orientation.x, tb.pose.orientation.y, tb.pose.orientation.z, tb.pose.orientation.w)
        w = tf.euler_from_quaternion(q)[2]
        v = math.sqrt(tb.twist.linear.x ** 2 + tb.twist.linear.y ** 2)
        x, y = translateGazeboToMap(tb.pose.position.x, tb.pose.position.y)
        return x, y, w, v
    except rospy.ServiceException, e:
        rospy.logerr("Could not get state of robot: %s" % e)
        return None, None, None, None


def translateGazeboToMap(gx, gy):
    return gx + X_MAP_TO_GAZEBO_TRANSLATION, gy + Y_MAP_TO_GAZEBO_TRANSLATION


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", dest="distance")
    parser.add_argument("-s", dest="speed")
    args = parser.parse_args()
    distance = args.distance
    speed = args.speed

    rospy.init_node('forward_test')
    ig = "P(V(1, do Forward({distance},{speed}) then 2),V(2, end)::nil)".format(distance=distance, speed=speed);

    client = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
    client.wait_for_server()

    goal = ig_action_msgs.msg.InstructionGraphGoal(order=ig)

    start_x, start_y, start_w, start_v = getTurtleBotState()
    print("start (x, y): ({x}, {y})".format(x=start_x, y=start_y))

    rospy.loginfo("Sending goal location ...")
    start_time = rospy.get_time()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(MAX_RUN_TIME))
    end_time = rospy.get_time()
    rospy.loginfo("Elapsed time: " + str(end_time - start_time))

    end_x, end_y, end_w, end_v = getTurtleBotState()
    print("end (x, y): ({x}, {y})".format(x=end_x, y=end_y))


if __name__ == "__main__":
    main()