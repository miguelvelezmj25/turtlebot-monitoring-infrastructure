#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

import monitors

ESTIMATE_POSE = 'estimate_pose'


def amcl_pose_callback(data, file):
    # data_time = str(data.header.stamp.secs) + '.' + str(data.header.stamp.nsecs)
    data_time = data.header.stamp.secs

    global amcl_current_time
    if amcl_current_time == data_time:
        return

    amcl_current_time = float(data_time)
    pose = data.pose.pose
    position = pose.position

    file.write("time: {} | x: {} | y: {}\n".format(amcl_current_time, position.x, position.y))


if __name__ == '__main__':
    rospy.init_node(ESTIMATE_POSE, anonymous=True)

    amcl_current_time = 0
    monitor_file = open(monitors.DATA_FOLDER + ESTIMATE_POSE + '.txt', "w", 0)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=amcl_pose_callback,
                     callback_args=monitor_file, queue_size=1)
    rospy.spin()
