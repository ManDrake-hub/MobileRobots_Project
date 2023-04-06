#! /usr/bin/python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    rospy.init_node("vel_publisher")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        rospy.loginfo("Start moving forward")
        move.linear.x = 0.26
        move.angular.z = 0
        pub.publish(move)
        rate.sleep()

try:
    talker()
except rospy.ROSInterruptException:
    pass