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
            if data == 'GO BACK':
                data = 'GO'+"_"+"BACK"
            rospy.loginfo(f'data {data} type {type(data)}')
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

def process_camera_rx_image(msg):
    #rospy.sleep(0.5)
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    decoded_objects = decode(gray)
    for obj in decoded_objects:
        data = obj.data.decode('utf-8')
        rect = obj.rect
            
        cv2.rectangle(img, (rect.left, rect.top), (rect.left + rect.width, rect.top + rect.height), (0, 0, 255), 2)
        cv2.putText(img, data, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        print(f"dx_{data}")
        #cv2.putText(frame, confidence, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        pub.publish(data)
    try:
        #cv2.namedWindow('rx')
        #cv2.imshow('rx', img)
        #cv2.waitKey(1)
        pass
    except CvBridgeError as e:
        print(e)

def process_camera_lx_image(msg):
    #rospy.sleep(0.5)
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    decoded_objects = decode(gray)
    for obj in decoded_objects:
        data = obj.data.decode('utf-8')
        rect = obj.rect
            
        cv2.rectangle(img, (rect.left, rect.top), (rect.left + rect.width, rect.top + rect.height), (0, 0, 255), 2)
        cv2.putText(img, data, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        print(f"sx_{data}")
        #cv2.putText(frame, confidence, (rect.left, rect.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        pub.publish(data)
    try:
        #cv2.namedWindow('lx')
        cv2.imshow('lx', img)
        cv2.waitKey(1)
        pass
    except CvBridgeError as e:
        print(e)

def camera_image_rx_callback(image_msg):
    process_camera_rx_image(image_msg)

def camera_image_lx_callback(image_msg):
    process_camera_lx_image(image_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('image_input', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        # SIMULATION
        video_stream()
        #sub = rospy.Subscriber('camera_image', CompressedImage, camera_image_callback)
        # REALITY
        """sub_left = rospy.Subscriber('camera/lx/image', CompressedImage, camera_image_lx_callback)
        rospy.sleep(0.5)
        sub_right = rospy.Subscriber('camera/rx/image', CompressedImage, camera_image_rx_callback)
        rospy.sleep(0.5)"""
        rospy.spin()
        #cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
