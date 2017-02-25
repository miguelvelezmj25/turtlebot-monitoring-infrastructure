from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import time
from math import radians

class Turnbot():
	_cmd_vel = None

	def __init__(self):
		self._cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

	def createNode(self):
		rospy.init_node('InstructionGraphs', anonymous=False)

	def subscribe(self):
		rospy.Subscriber("/odom", Odometry, self.callback)
		rospy.spin()

	def callback(self, message):
		print "Z axis value is %f", round(message.twist.twist.angular.z, 2)
		print "Y axis value is %f", round(message.twist.twist.angular.y, 2)
		print "X axis value is %f", round(message.twist.twist.angular.x, 2)
		rospy.sleep(0.2)
		if round(message.twist.twist.angular.z, 1)  != round(radians(90), 1):
			self.turn()

	def turn(self):
		#print "90 degrees is radians round 2 decimal places" % round(radians(90), 2)
		twist = Twist()
		twist.angular.z = radians(45) #0.785398*2*rotation    # 90 deg/s
		self._cmd_vel.publish(twist)
		#rospy.sleep(1)

	def turnLoop(self):
		print "90 degrees is radians round 2 decimal places", round(radians(90), 2)
		twist = Twist()
		r = rospy.Rate(5);
		twist.angular.z = radians(45) #0.785398*2*rotation    # 90 deg/s
		for i in range(0,10):
			self._cmd_vel.publish(twist)
			r.sleep()


if __name__ == "__main__":
	turnbot = Turnbot()
	turnbot.createNode()
	turnbot.subscribe()
	#turnbot.turnLoop()




