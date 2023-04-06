#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


pub = None
rate = None


def callback(data):
    global pub, rate
    # rospy.loginfo(data.ranges)
    front = data.ranges[0]
    back = data.ranges[180]
    right = data.ranges[90]
    left = data.ranges[270]

    min_dist = 0.5

    move = Twist()
    move.linear.x = 0.26
    move.angular.z = 0

    if front < min_dist:
        if left > min_dist:
            move.linear.x = 0.0
            move.angular.z = -0.26
        elif right > min_dist:
            move.linear.x = 0.0
            move.angular.z = 0.26
    
    pub.publish(move)
    #rate.sleep()


def listener():
    global pub, rate
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(0.1)
    rospy.Subscriber("/scan", LaserScan, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()