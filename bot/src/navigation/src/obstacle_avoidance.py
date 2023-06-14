#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(object):
    def __init__(self):
        rospy.init_node('escape_behavior')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.escape_distance = 0.65  # Set the maximum escape distance here
        self.escape_min = 0.1
        self.escape_speed = -0.2  # Set the backward escape speed here
        self.was_escaping = False

    def scan_callback(self, scan):
        ranges = scan.ranges

        front = ranges[0]
        back = ranges[180]

        if self.escape_min < front < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=False)
        elif self.escape_min < back < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=True)

    def stop(self):
        print("stop")
        twist = Twist()
        twist.linear.x = 0
        self.pub.publish(twist)

    def right(self):
        print("forward")
        twist = Twist()
        twist.angular.z = 0.26
        self.pub.publish(twist)

    def escape(self, reverse=False):
        print("escape")
        twist = Twist()
        twist.linear.x = self.escape_speed if not reverse else -self.escape_speed
        self.pub.publish(twist)

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    rospy.spin()
