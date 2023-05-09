#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

def video_stream():
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # decode QR code
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            data = obj.data.decode('utf-8')
            confidence = str(obj.orientation)
            rect = obj.rect
            
            cv2.rectangle(frame, (rect.left, rect.top), (rect.left + rect.width, rect.top + rect.height), (0, 0, 255), 2)
            cv2.putText(frame, data, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            #cv2.putText(frame, confidence, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            pub.publish(data)
        if ret:
            try:
                cv2.imshow('video_stream', frame)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('image_input', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        video_stream()
    except rospy.ROSInterruptException:
        pass
