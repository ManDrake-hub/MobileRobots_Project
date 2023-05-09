#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

##################### TO DO: AS SERVICE
def callback_qr(msg):
    command = msg.data.lower()
    commands[command] += 1
    most_common_command = max(commands, key=commands.get)
    print(f"Common command: {most_common_command}")
    for key in commands.keys():
            commands[key] = 0
    pub.publish()
    return most_common_command

if __name__ == '__main__':
    try:
        rospy.init_node('qr_common', anonymous=True)
        commands={"straight_on":0, "left":0, "right":0, "stop":0, "go_back":0}
        pub = rospy.Publisher("qr_most_common", String, queue_size=10)
        sub = rospy.Subscriber('qr_data_topic', String, callback_qr)
        rospy.sleep(3.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
