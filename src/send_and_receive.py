#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import rtde_control
import rtde_receive
import time

# from rtde_control import RTDEControlInterface as RTDEControl


def initialize_ur_rtde():
    global rtde_c, rtde_r
    rtde_c = rtde_control.RTDEControlInterface("172.31.1.144")
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    init_q = rtde_r.getActualQ()
    init_t = rtde_r.getActualTCPPose()
    print("Initialized")
    return init_q, init_t

def custom_movej(joint_angles):
    global rtde_c
    new_joints = joint_angles
    new_joints[5] = new_joints[5] + 0.2
    rtde_c.moveJ(new_joints, 1.05, 1.4, True)
    time.sleep(1.0)

    new_joints[5] = new_joints[5] - 0.2
    rtde_c.moveJ(new_joints, 1.05, 1.4, True)
    time.sleep(1.0)
    # Stop the movement before it reaches new_q
    rtde_c.stopJ(0.5)


def custom_movel(tcp_pose):
    global rtde_c
    new_target = tcp_pose
    new_target[0] = new_target[0] + 0.4
    rtde_c.moveL(new_target, 0.25, 0.5, True)
    time.sleep(1.0)
    rtde_c.stopL(0.5)

    new_target[0] = new_target[0] - 0.4
    rtde_c.moveL(new_target, 0.25, 0.5, True)
    time.sleep(2.0)

    # Stop the movement before it reaches target
    rtde_c.stopL(0.5)


def initialize_ros_node():
    # pub = rospy.Publisher('/current_pose', String, queue_size=10)
    joint_angles, tcp_pose = initialize_ur_rtde()
    rospy.init_node('send_and_receive', anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print(tcp_pose)
        # custom_movej(joint_angles)
        custom_movel(tcp_pose)
        rate.sleep()

if __name__ == '__main__':
    initialize_ros_node()