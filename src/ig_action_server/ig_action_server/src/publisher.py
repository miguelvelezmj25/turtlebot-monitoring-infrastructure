# The script is used to initialize node and publish topics.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

# Publishes status
PUB = rospy.Publisher('instructiongraphs_status',String, queue_size=10)

# Publishes goal
GOAL = rospy.Publisher('instructiongraphs_goal',String, queue_size=10)
IS_INIT = False

#For moving
CMD_VEL = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
MOVE_BASE = None

def shutdown(self):
  global CMD_VEL
  global MOVE_BASE
  rospy.loginfo('Stopping IG')
  # Make sure the robot stops on shutdown
  CMD_VEL.publish(Twist())
  if not MOVE_BASE is None:
    MOVE_BASE.cancel_goal()
  rospy.sleep(1)

def initialize():
  pass
  # global IS_INIT
  # global PUB
  # if not IS_INIT:
  #   IS_INIT = True
  #   rospy.init_node('InstructionGraphs', anonymous=False)
  #   rospy.on_shutdown(shutdown)
     # PUB = rospy.Publisher('instructiongraphs_status',String, queue_size=10)
  #   PUB.publish("Initialized")

def publish(msg):
  global IS_INIT
  global PUB
  if not IS_INIT:
    initialize()
  PUB.publish(msg)

def pub_goal(msg):
  global IS_INIT
  global GOAL
  if not IS_INIT:
    initialize()
  GOAL.publish(msg)

def pub_cmd_vel(msg):
  global IS_INIT
  global CMD_VEL
  CMD_VEL.publish (msg)
  rospy.sleep(0.5)

def move_base_action_client():
  #global IS_INIT
  global MOVE_BASE
  #if not IS_INIT:
  #  initialize()
  if MOVE_BASE is None:
    MOVE_BASE = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    MOVE_BASE.wait_for_server()
  return MOVE_BASE
  
def close_move_base_action_client():
  global MOVE_BASE
  if not MOVE_BASE is None:
    del MOVE_BASE
    MOVE_BASE = None
    

if __name__ == "__main__":
  initialize()
	
