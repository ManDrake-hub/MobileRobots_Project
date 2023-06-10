#! /usr/bin/python3
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node("stop")
    pub = rospy.Publisher("stop", Bool,queue_size=1)
    rospy.sleep(0.5)
    while True:
        input("If you want to stop the robot location press ENTER; after the movement press Ctrl+c")
        try:
            while True:
                pub.publish(True)
        except rospy.ROSInterruptException:
            pub.publish(False)