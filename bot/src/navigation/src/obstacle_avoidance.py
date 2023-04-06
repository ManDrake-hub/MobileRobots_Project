#! /usr/bin/python3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy

def callback(dt):
	front = dt.ranges[0]
	right = dt.ranges[15]
	left = dt.ranges[345]

	thr1 = 0.8
	thr2 = 0.8

	if front>thr1 and right>thr2 and left>thr2:
		rospy.loginfo("STate: GO ON")
		move.linear.x = 0.5
		move.angular.z = 0.0
	else:
		rospy.loginfo("STATE: ROTATE")
		move.linear.x = 0.0
		move.angular.z = 0.5
	pub.publish(move)

if(__name__=="__main__"):
	rospy.init_node("obstacle_avoidance")
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('scan', LaserScan, callback)
	move = Twist()
	rospy.spin()
