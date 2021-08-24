#!/usr/bin/env python3

# imports
import rospy
import numpy
from math import radians as d2r
from math import degrees as r2d
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, PoseStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import String

import sys
sys.path.insert(0, "/home/gizem/catkin_ws/src/my_human_pkg/src/Classes")
import Kinematics_with_Quaternions_py3 as kinematic

from rtde_control import RTDEControlInterface as RTDEControl

test_str = String()


def main():
    rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
    rospy.init_node('ur_rtde_test')
    pub = rospy.Publisher('/test_str', String, queue_size=1)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)
            test_str.data = "Gizem"
            print(test_str)
            pub.publish(test_str)
            rate.sleep()
        except KeyboardInterrupt:
            rtde_c.stopScript()
            raise


if __name__ == '__main__':
    main()
