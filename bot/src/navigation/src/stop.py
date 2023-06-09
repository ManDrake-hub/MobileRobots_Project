#! /usr/bin/python3
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("stop")
    pub = rospy.Publisher("stop", Bool,queue_size=1)
    rospy.sleep(0.5)
    while True:
        rospy.loginfo("If you want to change robot location, write stop and press ENTER")
        input()
        pub.publish(True)
        rospy.sleep(0.5)