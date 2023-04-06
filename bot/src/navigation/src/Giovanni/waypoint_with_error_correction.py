#! /usr/bin/python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import random


pub = None

waypoints = [
    (0, 0, 0),
    (5, 0, 0),
    (10, 0, 0)
]

def calculate_time(pos_start, pos_end, speed):
    pos_delta = pos_end - pos_start
    t = pos_delta / speed
    return t

def forward():
    global pub
    move = Twist()
    move.linear.x = 0.26
    move.angular.z = 0
    pub.publish(move)

def stop():
    global pub
    move = Twist()
    move.linear.x = 0.0
    move.angular.z = 0
    pub.publish(move)

def talker():
    global pub
    rospy.init_node("vel_publisher")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    rospy.sleep(1)

    for i in range(len(waypoints) - 1):
        forward()
        start = waypoints[i][0]
        end = waypoints[i+1][0] + random.random() * 1.0
        rospy.sleep(calculate_time(pos_start=start, pos_end=end, speed=0.26))
        stop()
        input("Press any button")

if __name__ == "__main__":
    talker()