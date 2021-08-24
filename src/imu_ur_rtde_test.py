#!/usr/bin/env python3

# imports
import rospy
import time
import numpy
from math import radians as d2r
from math import degrees as r2d
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, PoseStamped, Point
from std_msgs.msg import Float64

import sys
sys.path.insert(0, "/home/gizem/catkin_ws/src/my_human_pkg/src")
import Classes.Kinematics_with_Quaternions_py3 as kinematichuman_joint_imu
from Classes.IMU_class_single import IMUsubscriber

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

ur_cmd = Vector3()
diff_time = Float64()
prev = time.time()

diff_time_ur = Float64()
prev_ur = time.time()

def cb_test_move(msg):
    global prev
    now = time.time()
    diff_time = now-prev
    prev = now
    ur_cmd.y = msg.x*0.1
    ur_cmd.z = msg.y*0.1
    ur_cmd.x = msg.z*0.1
    print("diff_time:" + repr(diff_time))
    


def main():
    rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    rospy.init_node('ur_rtde_test')
    sub = rospy.Subscriber('/test_move_cmd', Vector3, cb_test_move)
    pub = rospy.Publisher('/ur_cmd', Vector3, queue_size=1)

    rate = rospy.Rate(1000)  # I can run as fastest as 125 Hz

    rtde_c.moveL([0.106, 0.715, -0.0, 2.616, -2.477, -1.510], 0.5, 0.3)
    target = rtde_r.getActualTCPPose()

    while not rospy.is_shutdown():
        global prev_ur
        try:
            now = time.time()
            diff_time_ur = now-prev_ur
            prev_ur = now
            print("ur_time:" + repr(diff_time_ur))
            # print(rtde_c.getActualJointPositionsHistory())
            # rtde_c.moveJ([d2r(92.70), d2r(-121.12), d2r(-92.18), d2r(-174.31), d2r(-87.61), d2r(-91.58)], 0.5, 0.3)
            # rtde_c.moveL([0.106+ur_cmd.x, 0.715+ur_cmd.y, 0.0+ur_cmd.z, 2.616, -2.477, -1.510], 0.5, 0.3)
            # print(ur_cmd)
            pub.publish(ur_cmd)
            rate.sleep()
        except KeyboardInterrupt:
            rtde_c.stopScript()
            raise


if __name__ == '__main__':
    main()
