#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def publish_camera_image():
    rospy.init_node('camera_publisher_node', anonymous=True)
    image_publisher = rospy.Publisher('camera_image', CompressedImage, queue_size=10)
    rate = rospy.Rate(10)  # Frequenza di pubblicazione dell'immagine (10 Hz)

    capture = cv2.VideoCapture(0)  # Apri la videocamera del computer

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret:
            # Converti l'immagine in formato CompressedImage
            compressed_image_msg = bridge.cv2_to_compressed_imgmsg(frame)
            image_publisher.publish(compressed_image_msg)
        rate.sleep()

    capture.release()

if __name__ == '__main__':
    try:
        publish_camera_image()
    except rospy.ROSInterruptException:
        pass
