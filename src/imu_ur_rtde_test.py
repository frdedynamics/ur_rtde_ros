#!/usr/bin/env python3

# imports
import rospy
import time
import numpy
from math import radians as d2r
from math import degrees as r2d
from tf.transformations import quaternion_from_euler as e2q
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Quaternion, PoseStamped, Point, Pose
from std_msgs.msg import Float64

import sys
sys.path.insert(0, "/home/gizem/catkin_ws/src/my_human_pkg/src")
import Classes.Kinematics_with_Quaternions_py3 as kinematichuman_joint_imu
from Classes.IMU_class_single import IMUsubscriber

from rtde_control import RTDEControlInterface as RTDEControl
import rtde_receive

ur_cmd = Vector3()
ur_cmd_init = Vector3()
diff_time = Float64()
prev = time.time()

diff_time_ur = Float64()
prev_ur = time.time()

home_approach_joints = [d2r(-95.72), d2r(-85.83), d2r(95.93), d2r(-3.86), d2r(95.29), d2r(-89.49)]

calib_flag = False

def cb_test_move(msg):
    global prev,calib_flag
    # now = time.time()
    # diff_time = now-prev
    # prev = now
    if not calib_flag:
        ur_cmd_init.x = msg.x
        ur_cmd_init.y = msg.y
        ur_cmd_init.z = msg.z
        calib_flag = True
    ur_cmd.x = d2r(msg.x-ur_cmd_init.x)
    ur_cmd.z = d2r(msg.y-ur_cmd_init.y)
    ur_cmd.y = 0
    # print("diff_time:" + repr(diff_time))


def list_to_pose(l):
    '''
    Convert [x,y,z,Rx,Ry,Rz] list as Pose(Point, Quaternion) object
    '''
    l_converted = Pose()
    l_converted.position.x = l[0]
    l_converted.position.y = l[1]
    l_converted.position.z = l[2]
    q = e2q(l[3], l[4], l[5] ,axes='rxyz')
    l_converted.orientation.x = q[0]
    l_converted.orientation.y = q[1]
    l_converted.orientation.z = q[2]
    l_converted.orientation.w = q[3]
    return l_converted
    


def main():
    rtde_c = RTDEControl("172.31.1.144", RTDEControl.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface("172.31.1.144")
    rospy.init_node('ur_rtde_test')
    sub = rospy.Subscriber('/sensor_r_elbow_rpy', Vector3, cb_test_move)
    # pub = rospy.Publisher('/ur_cmd', Vector3, queue_size=1)
    pub = rospy.Publisher('/ur_cmd', Pose, queue_size=1)

    rate = rospy.Rate(125)

    rtde_c.moveL([0.106, 0.715, -0.0, 2.616, -2.477, -1.510], 0.5, 0.3)
    t = rtde_r.getActualTCPPose()
    # rtde_c.moveL([t[0], t[1], t[2], t[3], t[4]+0.2, t[5]], 0.5, 0.3)
    print(t)

    dt = 1.0/500  # 2ms
    lookahead_time = 0.1
    gain = 300

    while not rospy.is_shutdown():
        global prev_ur
        try:
            pose_goal_list = rtde_c.getForwardKinematics(home_approach_joints)
            pose_goal = list_to_pose(pose_goal_list)
            pub.publish(pose_goal)
            rtde_c.moveJ(home_approach_joints, 1.05, 1.04, True)
            ########
            # now = time.time()
            # diff_time_ur = now-prev_ur
            # prev_ur = now
            # print("ur_time:" + repr(diff_time_ur))
            # # print(rtde_c.getActualJointPositionsHistory())
            # # rtde_c.moveJ([d2r(92.70), d2r(-121.12), d2r(-92.18), d2r(-174.31), d2r(-87.61), d2r(-91.58)], 0.5, 0.3)
            # # rtde_c.moveL([0.106+ur_cmd.x, 0.715+ur_cmd.y, 0.0+ur_cmd.z, 2.616, -2.477, -1.510], 0.5, 0.3)
            # rtde_c.servoL([0.106, 0.715, 0.0, 2.616+ur_cmd.x, -2.477+ur_cmd.y, -1.510+ur_cmd.z], 0.5, 0.3, dt, lookahead_time, gain)
            # # print(ur_cmd)
            # pub.publish(ur_cmd)
            ###########
            rate.sleep()
        except KeyboardInterrupt:
            rtde_c.stopScript()
            raise


if __name__ == '__main__':
    calib_flag = False
    main()
