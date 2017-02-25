import rospy
from geometry_msgs.msg import Twist, Quaternion
import tf
from move_base_msgs.msg import MoveBaseGoal
import publisher
import actionlib
from actionlib_msgs.msg import *
import math
from math import radians

SETUP_DONE = False

def setup():
  publisher.initialize()
  #SETUP_DONE = True
  #rospy.init_node("IG", anonymous=False)

def say(speech):
  if not SETUP_DONE: setup()
  rospy.loginfo(speech)

def move(distance, angular, speed, delta_y, rotation):
  if not SETUP_DONE: setup()

  rospy.sleep(1)
  
  # create a Twist message, fill it in to drive forward
  twist = Twist()
  if speed != 0:
    move_base = publisher.move_base_action_client()
    # go the full distance
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = distance
    goal.target_pose.pose.orientation.w = 1



    move_base.send_goal(goal)
    success = move_base.wait_for_result()
#    for i in range(int(distance*2)):
#      goal = MoveBaseGoal()
#      goal.target_pose.header.frame_id = 'base_link'
#      goal.target_pose.header.stamp = rospy.Time.now()
#      goal.target_pose.pose.position.x = 0.5 #3 meters
#      goal.target_pose.pose.orientation.w = 1.0 #go forward
      
#      move_base.send_goal(goal)

#      success = move_base.wait_for_result(rospy.Duration(10))

#      if not success:
#        move_base.cancel_goal()
        #rospy.loginfo("The base failed to move forward 3 meters for some reason")
#        break
#      else:
        # We made it!
#        state = move_base.get_state()
#        if state == GoalStatus.SUCCEEDED:
#          continue
    publisher.close_move_base_action_client()
  # create a twist message, fill it in to turn
  else:
    move_base = publisher.move_base_action_client()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0
    yaw = rotation * math.pi/180;
    q = tf.transformations.quaternion_from_euler(0,0,yaw)
    print "x = %.2f" % q[0], ", y= %.2f" % q[1],", z= %.2f"% q[2], ", w= %.2f" %q[3]
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    move_base.send_goal(goal)
    success = move_base.wait_for_result()
    publisher.close_move_base_action_client()
#    twist.angular.z = radians(90)*rotation #0.785398*2*rotation    # 90 deg/s
#    for i in range(int(2*angular)):
#      publisher.pub_cmd_vel (twist)



