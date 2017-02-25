#!/usr/bin/python
from orientation import Orientation
import rospy

if __name__ == '__main__':
	rospy.init_node("euler_orientation_test")
	orient = Orientation()
	while True:
		print "Raw yaw -> ", str(orient.getRawYaw())
		print "Corrected yaw -> ", str(orient.getCorrectedYaw())
		rospy.sleep(3)