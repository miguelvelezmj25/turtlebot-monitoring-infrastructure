#!/usr/bin/python

import rospy
import actionlib
import dynamic_reconfigure.client
import sys
import ast

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point


AMCL = 'amcl'
MAX_RUN_TIME = 80


def configure_amcl(parameters):
    client = dynamic_reconfigure.client.Client("amcl")
    return client.update_configuration(parameters)


def move_to_goal(x_goal, y_goal):
    # define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # wait for the action server to come up
    while not ac.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal
    goal.target_pose.pose.position = Point(x_goal, y_goal, 0)

    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    start_time = rospy.get_time()
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(MAX_RUN_TIME))
    end_time = rospy.get_time()
    rospy.loginfo("Elapsed time: " + str(end_time - start_time))

    if ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


if __name__ == '__main__':
    configurations = ast.literal_eval(sys.argv[-1])
    map_navigation = configurations['navigation']
    goal_reached = False

    target_x = map_navigation['target_x']
    target_y = map_navigation['target_y']

    rospy.init_node('map_navigation')

    if AMCL in configurations:
        rospy.loginfo(configure_amcl(configurations[AMCL]))

    goal_reached = move_to_goal(target_x, target_y)

    if goal_reached:
        rospy.loginfo("Congratulations!")
    else:
        rospy.loginfo("Hard Luck!")
