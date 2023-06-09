#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
from optparse import OptionParser
import asyncio
import numpy as np
import websockets


async def process_camera_image(websocket, path):
    cv2.namedWindow('Camera lx', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Camera rx', cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        image_buffer = await websocket.recv()
        nparr = np.frombuffer(image_buffer, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        camera_id = int(img[0, 0, 0])

        # TODO: This has to be changed
        if camera_id == 1:
            cv2.imshow('Camera lx', img[1:])
        elif camera_id == 2:
            cv2.imshow('Camera rx', img[1:])

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = gray[1:]

        qr_decoder = cv2.QRCodeDetector()
        # decode QR code
        try: 
            decoded_text, points, _ = qr_decoder.detectAndDecode(gray)
            if len(decoded_text)>0:
                rospy.loginfo('%d QR code: %s', camera_id, decoded_text)

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


if __name__ == '__main__':
    try:
        rospy.init_node('image_input', anonymous=True)
        pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
        pub_params = rospy.Publisher("parameter_camera", Int32MultiArray, queue_size=1)
        # REALITY
        start_server = websockets.serve(process_camera_image, '192.168.178.65', 8000)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()
    except rospy.ROSInterruptException:
        pass
