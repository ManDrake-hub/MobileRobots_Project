#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
import random

if __name__ == "__main__":
    rospy.init_node("test_command")
    commands = ["straight on", "left", "right", "go back"]
    pub_command = rospy.Publisher('qr_data_topic', String, queue_size=10)
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        pub_command.publish(random.choice(commands))
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)