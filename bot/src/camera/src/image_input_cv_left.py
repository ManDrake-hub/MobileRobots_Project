#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
from optparse import OptionParser

def process_camera_lx_image(msg):
    #rospy.sleep(0.5)
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    qr_decoder = cv2.QRCodeDetector()
        # decode QR code
    try: 
        decoded_text, points, _ = qr_decoder.detectAndDecode(gray)
        if len(decoded_text)>0:
            rospy.loginfo('LX QR code: %s', decoded_text)

            if len(points) > 0:
                points = points[0].astype(int)

                cv2.polylines(img, [points], True, (0, 255, 0), 2)
                cv2.putText(img, decoded_text, (points[0][0], points[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                pub.publish(decoded_text)
    except CvBridgeError as e:
            print(e)
    except Exception as e:
        pass
    cv2.imshow('lx', img)
    cv2.waitKey(1)

def camera_image_lx_callback(image_msg):
    process_camera_lx_image(image_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('image_input_lt', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        pub_params = rospy.Publisher("parameter_camera", Int32MultiArray, queue_size=1)
        # REALITY
        sub_left = rospy.Subscriber('camera/lx/image', CompressedImage, camera_image_lx_callback)
        rospy.sleep(0.5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass