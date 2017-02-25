#! /usr/bin/env python

import roslib; roslib.load_manifest('ig_action_msgs')
import rospy

import actionlib

import ig_action_msgs.msg

import sys

def ig_client():
	client = actionlib.SimpleActionClient("ig_action_server", ig_action_msgs.msg.InstructionGraphAction)
	client.wait_for_server()

	try:
		igfile = open(sys.argv[1], "r")
		igcode = igfile.read()
	except Exception as e:
		print e
		print "Could not open file for reading!"

	goal = ig_action_msgs.msg.InstructionGraphGoal(order=igcode)

	client.send_goal(goal)

	client.wait_for_result()

	rospy.loginfo('Result: %s' % (client.get_result()))

if __name__ == "__main__":
	try:
		rospy.init_node("ig_action_client")
		ig_client()
	except rospy.ROSInterruptException:
		rospy.loginfo("Encountered error!")