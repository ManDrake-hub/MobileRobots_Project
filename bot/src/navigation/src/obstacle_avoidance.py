#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(object):
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move = Twist()

    def scan_callback(self, scan):
        front = scan.ranges[0]
        right = scan.ranges[15]
        left = scan.ranges[345]

        thr1 = 0.8
        thr2 = 0.8

        if front<=thr1 and right<=thr2 and left<=thr2:
            rospy.loginfo("Obstacle, ROTATE")
            self.move.linear.x = 0.0
            self.move.angular.z = 0.5
        self.cmd_vel_pub.publish(self.move)


        
if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    rospy.spin()
