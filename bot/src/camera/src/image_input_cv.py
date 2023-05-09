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
    qr_decoder = cv2.QRCodeDetector()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # decode QR code
        decoded_text, points, _ = qr_decoder.detectAndDecode(frame)
        if points is not None:
            num_points = len(points[0])
            for i in range(num_points):
                pt1 = (int(points[0][i][0]), int(points[0][i][1]))
                pt2 = (int(points[0][(i+1) % num_points][0]), int(points[0][(i+1) % num_points][1]))

                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            
            cv2.putText(frame, decoded_text, (int(points[0][0][0]), int(points[0][0][1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            pub.publish(decoded_text)
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
        pub = rospy.Publisher('qr_data_topic', String, queue_size=10)
        video_stream()
    except rospy.ROSInterruptException:
        pass
