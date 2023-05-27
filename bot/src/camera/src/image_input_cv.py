#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from optparse import OptionParser

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
    #cap.release()
    #cv2.destroyAllWindows()

def process_camera_rx_image(msg):
    #rospy.sleep(0.5)
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    qr_decoder = cv2.QRCodeDetector()

    while not rospy.is_shutdown():
        # decode QR code
        try: 
            decoded_text, points, _ = qr_decoder.detectAndDecode(img)
            if len(decoded_text)>0:
                rospy.loginfo('RX QR code: %s', decoded_text)

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
    cv2.imshow('rx', img)
    cv2.waitKey(1)

def process_camera_lx_image(msg):
        #rospy.sleep(0.5)
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    qr_decoder = cv2.QRCodeDetector()

    while not rospy.is_shutdown():
        # decode QR code
        try: 
            decoded_text, points, _ = qr_decoder.detectAndDecode(img)
            if len(decoded_text)>0:
                rospy.loginfo('RX QR code: %s', decoded_text)

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

def camera_image_rx_callback(image_msg):
    process_camera_rx_image(image_msg)

def camera_image_lx_callback(image_msg):
    process_camera_lx_image(image_msg)

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--mode", dest="mode", default="0")
    (options, args) = parser.parse_args()
    mode = int(options.mode)

    try:
        rospy.init_node('image_input', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        if mode == 0:
            # SIMULATION
            video_stream()
        else:
            # REALITY
            sub_left = rospy.Subscriber('camera/lx/image', CompressedImage, camera_image_lx_callback)
            rospy.sleep(0.5)
            sub_right = rospy.Subscriber('camera/rx/image', CompressedImage, camera_image_rx_callback)
            rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
