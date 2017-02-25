#!/usr/bin/python
##############################################
## The python code continuously reads the current
## yaw and can be used by other modules to provide
## the same information.
##############################################
import roslib; roslib.load_manifest('messages')
import rospy
import tf
import time

from messages.msg import euler

class Orientation():
	_drift_constant = 0.019
	_in_unit_time = 3.0
	def __init__(self):
		pass

	def getCorrectedYaw(self, init_time, init_yaw, current_time, current_yaw):
		print "init_time - " + str(init_time) + ", init_yaw - " + str(init_yaw)
		print "current_time - " + str(current_time) + ", current_yaw - " + str(current_yaw)
		difference = current_time - init_time
		if difference <= 0.0:
			return  current_yaw
		else:
			total_drift = difference / self._in_unit_time * self._drift_constant
			corect_yaw = current_yaw + total_drift
			return corect_yaw

if __name__ == '__main__':
	rospy.init_node("orientations")
	Orientation()