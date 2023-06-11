#! /usr/bin/python3
import rospy
import cv2
from std_msgs.msg import String, Int32MultiArray
import asyncio
import numpy as np
import websockets

def closest(value: np.ndarray):
    _value = value.astype(np.int16)
    dist_0 = np.sum(np.abs(np.full_like(_value, fill_value=0) - _value))
    dist_255 = np.sum(np.abs(np.full_like(_value, fill_value=255) - _value))
    if dist_255 < dist_0:
        return 1
    return 0

async def process_camera_image(websocket, path):
    while True:
        image_buffer = await websocket.recv()
        nparr = np.frombuffer(image_buffer, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        camera_id = closest(img[0, 0])

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        qr_decoder = cv2.QRCodeDetector()
        # decode QR code
        decoded_text, points, _ = qr_decoder.detectAndDecode(gray)
        if len(decoded_text)>0:
            rospy.loginfo('%d QR code: %s', camera_id, decoded_text)

            if len(points) > 0:
                points = points[0].astype(int)

                cv2.polylines(img, [points], True, (0, 255, 0), 2)
                cv2.putText(img, decoded_text, (points[0][0], points[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                pub.publish(decoded_text)
        print(f"published{camera_id}")
        cv2.imshow('Camera lx' if camera_id == 0 else 'Camera rx', img)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_input', anonymous=True)
    pub = rospy.Publisher('qr_data_topic', String, queue_size=1)
    pub_params = rospy.Publisher("parameter_camera", Int32MultiArray, queue_size=1)
    # REALITY
    start_server = websockets.serve(process_camera_image, 'localhost', 8000, ping_interval=None)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
