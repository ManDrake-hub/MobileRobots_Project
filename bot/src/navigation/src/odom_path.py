#! /usr/bin/python3
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rospy

class MyPlotPath:
	def __init__(self):
		rospy.init_node("path node")
		self.odom_path = Path()
	
	def odom_callback(self,data):
		self.odom_path.header = data.header
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose
		self.odom_path.poses.append(pose)
		self.path_pub.publish(self.odom_path)


if(__name__=="__main__"):
	try:
		p = MyPlotPath()
		p.odom_sub = rospy.Subscriber("odom", Odometry,p.odom_callback)
		p.path_pub = rospy.Publisher("odom_path",Path,queue_size=10)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
