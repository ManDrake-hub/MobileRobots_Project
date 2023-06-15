#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from camera.srv import QR,QRResponse

command = None

def callback_qr(msg):
    global command
    command = msg.data.lower().replace("\u200b", "").replace("_", " ")
    print(f"COMMAND:{command}")

def get_next_command(req):
    rospy.loginfo('search QR')
    global command
    response = QRResponse()
    if command is None:
        print("QR code not found")
        response.answer.data = ""
    else:
        response.answer.data = command
    command = None
    return response

if __name__ == '__main__':
    try:
        rospy.init_node('qr_service', anonymous=True)
        sub = rospy.Subscriber('qr_data_topic', String, callback_qr)
        s = rospy.Service('QR_command', QR, get_next_command)
        rospy.sleep(3.0)
        rospy.spin()
    except Exception as e:
        print(e)
