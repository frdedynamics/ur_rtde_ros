#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from rtde_control import RTDEControlInterface as RTDEControl

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    rtde_c = RTDEControl("172.31.1.144")
    speed = [0, 0, 0.100, 0, 0, 0]
    rtde_c.moveUntilContact(speed)

    rtde_c.stopScript()

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
   talker()
