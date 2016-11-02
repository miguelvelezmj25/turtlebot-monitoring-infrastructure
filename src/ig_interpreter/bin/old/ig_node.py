import rospy
import parse_and_execute
from geometry_msgs.msg import Twist

def shutdown(self):
  rospy.loginfo('Stopping IG')
  # Make sure the robot stops on shutdown
  self.cmd_vel.publish(Twist())
  rospy.sleep(1)

cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)

rospy.init_node ('IG')
rospy.on_shutdown(self.shutdown)

ig = rospy.get_param('~ig_file')

print ('Got ig_file as ' + ig)

try:
  parse_and_execute(ig, self)
except Exception as e:
  print "Could not open file '" + ig + "' for reading"
  exit(1)


rospy.spin();
