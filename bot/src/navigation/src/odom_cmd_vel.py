#! /usr/bin/python3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy

class MyNode:
	def __init__(self):
		rate = rospy.Rate(1)
		rospy.Subscriber('odom',Odometry,self.callback_odom)
		rospy.Subscriber('cmd_vel',Twist,self.callback_command)
		rospy.spin()
	
	def callback_odom(self,odom):
		rospy.loginfo("ODOMETRY: Lin, Ang")
		rospy.loginfo(odom.twist.twist.linear.x)
		rospy.loginfo(odom.twist.twist.angular.z)
		rospy.loginfo("COVARIANCE ODOMETRY")
		rospy.loginfo(odom.twist.covariance)
	
	def callback_command(self,twist):
		rospy.loginfo("COMMAND: Lin, Ang")
		rospy.loginfo(twist.linear.x)
		rospy.loginfo(twist.angular.z)


if(__name__=="__main__"):
	rospy.init_node("OdomNode")
	try:
		node = MyNode()
	except rospy.ROSInterruptException:
		pass
