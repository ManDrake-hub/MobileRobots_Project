#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(object):
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.move = Twist()

    def scan_callback(self, scan):
        front = scan.ranges[0]
        right = scan.ranges[15]
        left = scan.ranges[345]

        thr_min = 0.4
        thr_max = 0.8

        if thr_min<=front<=thr_max or thr_min<=right<=thr_max or thr_min<=left<=thr_max:
            rospy.loginfo("Obstacle, ROTATE")
            self.move.linear.x = 0
            self.move.linear.y = 0
            self.move.angular.z = 0.5
            self.cmd_vel_pub.publish(self.move)

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    rospy.spin()
