#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import rtde_control
import rtde_receive
import time

# from rtde_control import RTDEControlInterface as RTDEControl


def initialize_ur_rtde():
    rtde_c = rtde_control.RTDEControlInterface("172.31.1.144")
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    init_q = rtde_r.getActualQ()
    return init_q


def initialize_ros_node():
    # pub = rospy.Publisher('/current_pose', String, queue_size=10)
    current_pose = initialize_ur_rtde()
    rospy.init_node('send_and_receive', anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print(current_pose)
        rate.sleep()

if __name__ == '__main__':
    initialize_ros_node()