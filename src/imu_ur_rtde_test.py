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
    rtde_c = RTDEControl("172.31.1.144")
    rospy.init_node('ur_rtde_test')
    pub = rospy.Publisher('/test_str', String, queue_size=1)
    rate = rospy.Rate(50)

    velocity = 0.5
    acceleration = 0.5
    blend_1 = 0.0
    blend_2 = 0.02
    blend_3 = 0.0
    path_pose1 = [0.431, 0.633, 0.035, 4.384, -1.320, 1.148, velocity, acceleration, blend_1]
    path_pose2 = [0.119, 0.888, -0.006, 4.738, -0.354, 0.113, velocity, acceleration, blend_2]
    path_pose3 = [-0.349, 0.752, 0.163, 4.635, 0.362, -0.448, velocity, acceleration, blend_3]
    path = [path_pose1, path_pose2, path_pose3]

    while not rospy.is_shutdown():
        try:
            rtde_c.moveL(path)
            test_str.data = "Gizem"
            print(test_str)
            pub.publish(test_str)
            rate.sleep()
        except KeyboardInterrupt:
            rtde_c.stopScript()
            raise


if __name__ == '__main__':
    main()
