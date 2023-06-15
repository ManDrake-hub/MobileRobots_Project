#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
import random

# If you want to use cameras for QR detection, you can write the command on terminal
if __name__ == "__main__":
    rospy.init_node("test_command")
    pub_command = rospy.Publisher('qr_data_topic', String, queue_size=10)
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        command = input("Insert a command: ")
        pub_command.publish(command)
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)