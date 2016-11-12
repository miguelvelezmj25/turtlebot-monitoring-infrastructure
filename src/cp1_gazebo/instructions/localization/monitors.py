#!/usr/bin/python

import subprocess
import rospy
import os

DATA_FOLDER = 'monitor_data/'
MONITORS_FILE = 'monitors_file.txt'
MONITORS_CONFIG = '.monitorsconfig'


def cleanup_files():
    all_monitors_file_name = DATA_FOLDER + MONITORS_FILE

    with open(all_monitors_file_name, 'r', 0) as all_monitors_file:
        data = all_monitors_file.read().splitlines()

        for data_file_name in data:
            with open(DATA_FOLDER + data_file_name, 'r', 0) as data_file:
                data = data_file.read().splitlines()

            with open(DATA_FOLDER + data_file_name, 'w', 0) as data_file:
                i = 1

                while i < len(data):
                    data_file.write(data[i] + '\n')
                    i += 1


if __name__ == '__main__':
    rospy.init_node('monitors_manager', anonymous=True)

    if not os.path.exists(DATA_FOLDER):
        os.mkdir(DATA_FOLDER)

    # TODO have not tested if this works with cpp
    with open(MONITORS_CONFIG, 'r', 0) as monitor_config:
        monitors = monitor_config.read().splitlines()

    all_monitors_file = open(DATA_FOLDER + MONITORS_FILE, "w", 0)

    for monitor in monitors:
        all_monitors_file.write(monitor.split('.')[0] + ".txt\n")

    all_monitors_file.close()

    for monitor in monitors:
        subprocess.Popen("rosrun cp1_gazebo " + monitor, shell=True)

    rospy.spin()
