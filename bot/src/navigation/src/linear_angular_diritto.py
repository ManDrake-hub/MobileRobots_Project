#! /usr/bin/python3
from geometry_msgs.msg import Twist
import rospy

if(__name__=="__main__"):
	rospy.init_node("navigation")
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	msg = Twist()
	msg.linear.x = 0.26
	msg.angular.z = 0
	
	pub.publish(msg)

	rospy.spin()
