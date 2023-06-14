#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(object):
    '''
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
    '''
    def __init__(self):
        rospy.init_node('escape_behavior')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.escape_distance = 0.5  # Set the maximum escape distance here
        self.escape_speed = -0.2  # Set the backward escape speed here
        self.was_escaping = False

    def scan_callback(self, scan):
        ranges = scan.ranges

        front = ranges[0]
        back = ranges[180]

        if front < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=False)
        elif back < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=True)
        elif self.was_escaping:
            self.was_escaping = False
            for i in range(10):
                self.stop()
                rospy.sleep(0.1)
            for i in range(10):
                self.forward()
                rospy.sleep(0.1)

    def stop(self):
        print("stop")
        twist = Twist()
        twist.linear.x = 0
        self.pub.publish(twist)

    def forward(self):
        print("forward")
        twist = Twist()
        twist.linear.x = 0.26
        self.pub.publish(twist)

    def escape(self, reverse=False):
        print("escape")
        twist = Twist()
        twist.linear.x = self.escape_speed if not reverse else -self.escape_speed
        self.pub.publish(twist)

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    rospy.spin()
