import rospy
import time
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import publisher
from math import radians
from math import degrees
import dynamic_reconfigure.client
# from orientation import Orientation
import tf

def moveAbs(x,y,v):
	return move (x,y,v,"Absolute");

def moveRel(x,y,v):
	return move (x,y,v,"Relative");

def turnAbs(d,r, init_yaw, tf_listener):
	return turn2(d, r, init_yaw, tf_listener)

def turnRel(a,r):
	return turn(a,r)

def move(x,y,v,action):
	print "MOVING TO x:" + str(x) + " y:" + str(y)
	status=True
	msg = "Successfully executed the vertex"
	setVelocity(v, 'LINEAR');
	if action == "Absolute":
		frameType = "map"
	else:
		frameType = 'base_link'

	move_base = publisher.move_base_action_client ()
    
	print "moving ->" + str(frameType)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = frameType
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x 
	if frameType == 'map':
		goal.target_pose.pose.position.y = y #3 meters
	
	goal.target_pose.pose.orientation.w = 1.0 #go forward

	move_base.send_goal(goal)
	success = move_base.wait_for_result()
        
	if not success:
		move_base.cancel_goal()
		rospy.logerr("The base failed to move forward")
		msg = "The base failed to move forward"
		status = False
	else:
		succeeded = move_base.get_state () == GoalStatus.SUCCEEDED
		if not succeeded:
			rospy.logerr ("The base failed to move, status=%s" %succeeded)
			msg = "Move %s failed" %action
			status = False
		else:
			rospy.loginfo("Successfully executed the vertex")
	
	publisher.close_move_base_action_client()
	return status, msg

def turn(angle, rotation):
	status=True
	msg ="Turned successfully"
	#setVelocity(rotation, 'ANGULAR');
	twist = Twist()
	cmd_vel = getCmdVel()

	# listener = tf.TransformListener()
	# listener.waitForTransform("/base_link", "/odom_combined", rospy.Time(0), rospy.Duration(1))
	# (stamped_t,stamped_r) = listener.lookupTransform("/base_link", "/odom_combined", rospy.Time(0))
	# stamped = tf.Transform(stamped_t,stamped_r)
	# twist.z = rotation
	# rate = rospy.Rate(10)
	# done = False
	# while not done:
	# 	cmd_vel.publish(twist)
	# 	rate.sleep()
	# 	try:
	# 		(curr_t, curr_r) = listener.lookupTransform("/base_link", "/odom_combined", rospy.Time(0))
	# 		current = tf.Transform(curr_t, curr_r)
	# 		relative = stamped.inverse () * current
	# 		double rot_moved = relative.

	cycles = int(angle/45)
	twist.angular.z = radians(45)*cycles #0.785398*2*rotation    # 90 deg/s
	for i in range(0,int(cycles)):
		cmd_vel.publish(twist)
		rospy.sleep(0.5)
	return status,msg

def turn2(d, r, init_time, current_yaw, tf_listener):
	status=True
	msg="Turned successfully"
	if current_yaw == None:
		rospy.logerr("Initialization time is None, the startup was not correctly done!")
		msg="Initialization time is None, the startup was not correctly done!"
		return False, msg
	else:
		EAST = 0
		NORTH = radians(90)
		WEST = radians(180)
		SOUTH = radians(270)
		target_angle = EAST
		if d == 'NORTH':
			target_angle = NORTH
		elif d == 'SOUTH':
			target_angle = SOUTH
		elif d == 'EAST':
			target_angle = EAST
		elif d == 'WEST':
			target_angle = WEST
		else:
			rospy.logerr("Direction is not correct")
			return False

		rot = init_yaw - target_angle
		a2 = target_angle - init_yaw
		if abs(a2) < abs(rot):
			rot = a2
		return turn(degrees(angle),r)
		# print "Direction_yaw ->" + str(init_yaw)
		# if correct_yaw > init_yaw:
		# 	angle = correct_yaw - init_yaw
		# 	return turn(degrees(angle), -1)
		# else:
		# 	angle = init_yaw - correct_yaw
		# 	return turn(degrees(angle), 1)

def getCmdVel():
	cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
	rospy.sleep(1)
	return cmd_vel


def setVelocity(velocity, type):
	client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
	if type == 'LINEAR':
		params = {'max_vel_x' : velocity, 'max_trans_vel': velocity}
	else:
		params = {'max_rot_vel': velocity, 'min_rot_vel':velocity}
	config = client.update_configuration(params)
	rospy.sleep(0.2)
	rospy.loginfo("Velocity set to " + str(velocity))
