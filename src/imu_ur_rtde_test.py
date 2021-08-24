#!/usr/bin/env python3

# imports
import rospy
import numpy
from math import radians as d2r
from math import degrees as r2d
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, PoseStamped, Point
from std_msgs.msg import String

import sys
sys.path.insert(0, "/home/gizem/catkin_ws/src/my_human_pkg/src/Classes")
import Kinematics_with_Quaternions_py3 as kinematichuman_joint_imu
from IMU_class_single import IMUsubscriber

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

ur_cmd = Vector3()

def cb_test_move(msg):
    ur_cmd.y = msg.x
    ur_cmd.z = msg.y
    ur_cmd.x = msg.z


def main():
    rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    rospy.init_node('ur_rtde_test')
    pub = rospy.Publisher('/test_str', Vector3, queue_size=1)
    sub = rospy.Subscriber('/test_move_cmd', Vector3, cb_test_move)

    test_str = String()
    myIMU = IMUsubscriber()
    myIMU.init_subscribers_and_publishers()

    rate = rospy.Rate(50)


    rtde_c.moveL([0.106, 0.715, -0.0, 2.616, -2.477, -1.510], 0.5, 0.3)
    target = rtde_r.getActualTCPPose()

    while not rospy.is_shutdown():
        try:
            # print(rtde_c.getActualJointPositionsHistory())
            # rtde_c.moveJ([d2r(92.70), d2r(-121.12), d2r(-92.18), d2r(-174.31), d2r(-87.61), d2r(-91.58)], 0.5, 0.3)
            rtde_c.moveL([0.106+ur_cmd.x, 0.715+ur_cmd.y, 0.0+ur_cmd.z, 2.616, -2.477, -1.510], 0.5, 0.3)
            print(ur_cmd)
            pub.publish(ur_cmd)
            myIMU.update()
            rate.sleep()
        except KeyboardInterrupt:
            rtde_c.stopScript()
            raise


if __name__ == '__main__':
    main()
