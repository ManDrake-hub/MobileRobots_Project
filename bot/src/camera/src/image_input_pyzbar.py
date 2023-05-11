#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np

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

def process_camera_image(msg):
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    decoded_objects = decode(gray)
    for obj in decoded_objects:
        data = obj.data.decode('utf-8')
        rect = obj.rect
            
        cv2.rectangle(img, (rect.left, rect.top), (rect.left + rect.width, rect.top + rect.height), (0, 0, 255), 2)
        cv2.putText(img, data, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #cv2.putText(frame, confidence, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        pub.publish(data)
    try:
        cv2.imshow('video_stream', img)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def camera_image_callback(image_msg):
    process_camera_image(image_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('image_input', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        # SIMULATION
        #video_stream()
        sub = rospy.Subscriber('camera_image', CompressedImage, camera_image_callback)
        # REALITY
        #sub_left = rospy.Subscriber('camera/lx/image', CompressedImage, camera_image_callback)
        #sub_right = rospy.Subscriber('camera/rx/image', CompressedImage, camera_image_callback)
        rospy.spin()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
