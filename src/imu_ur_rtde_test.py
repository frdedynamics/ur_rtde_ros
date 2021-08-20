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


'''
class IMU:
    def __init__(self):
        self.ori_init = Imu()
        self.ori = Quaternion(0., 0., 0., 1.)
        self.calibration_flag = 0
        self.t_prev = 0.0
        self.ori_prev = Quaternion(0., 0., 0., 1.)
        self.vel = Quaternion(0., 0., 0., 0.) ## this has to be a pure quaternion. Only vectoral representation
        ## if subscribed as IMU
        # init_quat = Quaternion()
        # gyro = Vector3()
        # acc = Vector3()

my_imu = IMU()
my_stamped_pose = PoseStamped()


def cb_imu(msg):  # subscribed as IMU, not a RPY (Imu)
    while my_imu.calibration_flag < 20:
        my_imu.ori_init = msg.orientation
        print("calibrating")
        print(my_imu.calibration_flag)
    my_imu.ori = kinematic.q_multiply(kinematic.q_invert(my_imu.ori_init), msg.orientation)
    t_curr = msg.header.stamp.secs+msg.header.stamp.nsecs*10**(-9)
    dt = t_curr - my_imu.t_prev
    my_imu.vel = kinematic.q_multiply(kinematic.q_invert(my_imu.ori_prev), my_imu.ori) / dt
    my_imu.t_prev = t_curr
    my_imu.ori_prev = my_imu.ori


def imu_to_pose():
    my_stamped_pose.pose.orientation = my_imu.ori


def main():
    rospy.init_node('single_imu_ur_servo')
    pub = rospy.Publisher('/servo_server/target_pose', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('/sensor_r_wrist', Imu, cb_imu)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        my_stamped_pose.header.stamp =  rospy.Time.now()
        my_stamped_pose.header.frame_id = "world"
        my_stamped_pose.pose.position = Point(-0.43631838811541523, 0.13214048861608407, 0.8351070253889343)
        imu_to_pose()
        print(my_stamped_pose)
        my_imu.calibration_flag = my_imu.calibration_flag + 1
        pub.publish(my_stamped_pose)
        rate.sleep()

if __name__ == '__main__':
    main()
'''