#! /usr/bin/python3
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("stop")
    pub = rospy.Publisher("stop", Bool,queue_size=1)
    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    
    while True:
        counter = 0
        input("If you want to stop the robot location press ENTER; after the movement press ENTER")
        while counter < 30:
            pub.publish(True)
            counter += 1
            rate.sleep()
        input()
        pub.publish(False)