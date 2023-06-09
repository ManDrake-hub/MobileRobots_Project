#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError

def video_stream():
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)
    qr_decoder = cv2.QRCodeDetector()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # decode QR code
        if ret:
            try:
                decoded_text, points, _ = qr_decoder.detectAndDecode(frame)
                if len(decoded_text)>0:
                    rospy.loginfo('Decoded QR code: %s', decoded_text)

                    if len(points) > 0:
                        points = points[0].astype(int)

                        cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                        cv2.putText(frame, decoded_text, (points[0][0], points[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        pub.publish(decoded_text)

            except CvBridgeError as e:
                print(e)
            except Exception as e:
                pass
        cv2.imshow('video_stream', frame)
        cv2.waitKey(1)

# Run the estimation function
if __name__ == '__main__':
    rospy.init_node('optical_flow_qr', anonymous=True)
    pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
    video_stream()
    rospy.spin()
