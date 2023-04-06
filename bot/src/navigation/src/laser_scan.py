#! /usr/bin/python3
from sensor_msgs.msg import LaserScan
import rospy

def callback(msg):
	rospy.loginfo('>>>>>>>>>>>')
	rospy.loginfo('s1[0]')
	rospy.loginfo(msg.ranges[0])

	rospy.loginfo('s2[90]')
	rospy.loginfo(msg.ranges[90])

	rospy.loginfo('s3[180]')
	rospy.loginfo(msg.ranges[180])

	rospy.loginfo('s4[270]')
	rospy.loginfo(msg.ranges[270])

if(__name__=="__main__"):
	rospy.init_node("laser_data")
	sub = rospy.Subscriber('scan', LaserScan, callback)
	rospy.spin()
