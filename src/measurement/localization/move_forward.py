#!/usr/bin/python
import roslib;

roslib.load_manifest('ig_action_msgs')
import rospy

import actionlib
import ig_action_msgs.msg
import sys
import threading

import requests

import argparse


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

    client.send_goal(goal)

    client.wait_for_result()


if __name__ == "__main__":
    main()