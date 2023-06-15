#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ObstacleAvoidanceNode(object):
    def __init__(self):
        rospy.init_node('escape_behavior')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_escape = rospy.Subscriber('stop', Bool, self.stop_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.escape_distance = 0.65  # Set the maximum escape distance here
        self.escape_min = 0.1
        self.escape_speed = -0.2  # Set the backward escape speed here
        self.was_escaping = False
        self.stopped = False

    def stop_callback(self, msg):
        '''
        Callback function for the 'stop' topic.

        Args:
            msg: The message received on the 'stop' topic. True if the robot is stopped, else False
        '''
        self.stopped = msg

    def scan_callback(self, scan):
        '''
        Callback function for the '/scan' topic. Make the escape behaviour if the laser scan detect near obstacle front and back

        Args:
            scan: The laser scan data.
        '''
        if self.stopped:
            return

        ranges = scan.ranges

        front = ranges[0]
        back = ranges[180]

        if self.escape_min < front < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=False)
        elif self.escape_min < back < self.escape_distance:
            self.was_escaping = True
            self.escape(reverse=True)

    def escape(self, reverse=False):
        '''
        Initiates the escape behavior.

        Args:
            reverse: Determines whether to escape in reverse or forward. Defaults to False.
        '''
        print("escape")
        twist = Twist()
        twist.linear.x = self.escape_speed if not reverse else -self.escape_speed
        self.pub.publish(twist)

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    rospy.spin()
