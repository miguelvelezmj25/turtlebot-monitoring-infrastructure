import dynamic_reconfigure.client
import rospy

def reconfig():
	rospy.init_node('my_reconfigure', anonymous=False)
	client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
	params = {'min_vel_x' : '0.5', 'max_vel_x' : '1.0'}
	config = client.update_configuration(params)

if __name__ == '__main__':
	reconfig();
