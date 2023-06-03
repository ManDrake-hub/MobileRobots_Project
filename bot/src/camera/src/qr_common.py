#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from camera.srv import QR,QRResponse
from move_base_msgs.msg import MoveBaseActionResult
last_command = None
command = None
##################### TO DO: AS SERVICE
def callback_qr(msg):
    global last_command, command
    command =  msg.data.lower().replace("\u200b","")
    """most_common_command = max(commands, key=commands.get)
    print(f"Common command: {most_common_command}")
    for key in commands.keys():
            commands[key] = 0
    pub.publish()
    return most_common_command"""

def get_next_command(req):
    rospy.loginfo('SONO SOTTOSCRITTO AI QR')
    #rospy.wait_for_message(topic='move_base/result',topic_type=MoveBaseActionResult)
    global command, last_command
    last_command =  command
    command = None
    response = QRResponse()
    if last_command is None:
        print(f"QR code not found")
        response.answer.data = ""
    else:
        response.answer.data = last_command
    return response

if __name__ == '__main__':
    try:
        rospy.init_node('qr_common', anonymous=True)
        sub = rospy.Subscriber('qr_data_topic', String, callback_qr)
        s = rospy.Service('QR_command', QR, get_next_command)
        rospy.loginfo("QR command service ready.")
        rospy.sleep(3.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
