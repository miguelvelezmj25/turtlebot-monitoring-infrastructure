#!/usr/bin/python

import rospy
from gazebo_msgs.msg import ModelStates

import monitors

GROUND_TRUTH_POSE = 'ground_truth_pose'


def gazebo_model_states_callback(data, file):
    # data_time = rospy.get_rostime().to_sec()
    data_time = rospy.get_rostime().secs

    global gazebo_current_time
    if gazebo_current_time == data_time:
        return

    gazebo_current_time = data_time
    pose = data.pose[2]
    position = pose.position

    file.write("time: {} | x: {} | y: {}\n".format(gazebo_current_time, position.x, position.y))


if __name__ == '__main__':
    rospy.init_node(GROUND_TRUTH_POSE, anonymous=True)

    gazebo_current_time = 0
    monitor_file = open(monitors.DATA_FOLDER + GROUND_TRUTH_POSE + '.txt', "w", 0)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback=gazebo_model_states_callback,
                     callback_args=monitor_file, queue_size=1)
    rospy.spin()
