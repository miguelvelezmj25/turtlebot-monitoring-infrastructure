import rospy
from geometry_msgs.msg import Twist

SETUP_DONE = False

def setup():
  SETUP_DONE = True
  rospy.init_node("IG", anonymous=False)

def say(speech):
  if not SETUP_DONE: setup()
  rospy.loginfo(speech)

def move(distance, angular):
  if not SETUP_DONE: setup()
  cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
  r = rospy.Rate(10) # sleep 0.1 at a time
  move_cmd = Twist()
  move_cmd.linear.x = 0.5 # m/s
  move_cmd.angular.z = angular # rad/s

  for i in xrange(int(2*distance)):
    cmd_vel.publish(move_cmd)
    r.sleep()
