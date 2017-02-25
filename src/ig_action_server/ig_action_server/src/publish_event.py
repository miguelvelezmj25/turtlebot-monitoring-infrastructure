import roslib; roslib.load_manifest('ig_action_msgs')

import ig_action_msgs.msg

from ig_server import IGServer 

import rospy

def publish(asa):
	rospy.loginfo('IN publish events')
	feedback = ig_action_msgs.msg.InstructionGraphFeedback()
	feedback.sequence = "World hello!"
	asa._as.publish_feedback(feedback)
	