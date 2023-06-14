#! /usr/bin/python3
import rospy
import cv2 as cv
import asyncio
import websockets
from std_msgs.msg import String

class Node:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME, anonymous=True)
        self.qr_decoder = cv.QRCodeDetector()
        self.pub = rospy.Publisher('qr_data_topic', String, queue_size=30)
        rospy.sleep(0.5)
        self.camera_ids = {rospy.get_param('~camera_lx'): 0, rospy.get_param('~camera_rx'): 255}
        loop = asyncio.get_event_loop()
        loop.create_task(self.publish_frame_cb(rospy.get_param('~camera_rx')))
        loop.run_forever()

    def set_camera(self, cap):
        '''
        These lines of code are setting the properties of the camera stream using OpenCV's
        cv.VideoCapture()` method. Specifically, it is setting the frames per second (FPS) of the camera
        stream to the value specified in the ROS parameter `/camera/fps_capture`, and setting the frame
        width and height to the values specified in the ROS parameters `/camera/width` and `/camera/height`,
        respectively.
        '''
        self.width = rospy.get_param("/camera/width")
        self.height = rospy.get_param("/camera/height")
        cap.set(cv.CAP_PROP_FPS, rospy.get_param("/camera/fps_capture"))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)    
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

    def print_camera_info(self, cap):
        '''
        These lines of code are retrieving the width, height, and frames per second (FPS) of the
        camera stream using OpenCV's `cv.VideoCapture()` method. The values are then logged using
        `rospy.loginfo()`.
        '''
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv.CAP_PROP_FPS)
        rospy.loginfo(f"Frame size width x height: {int(width)}x{int(height)}@{fps} for camera")
    
    async def publish_frame_cb(self, camera_id):
        '''
        This code is running in a separate thread and continuously checking if the ROS node is still running
        using `rospy.is_shutdown()`. If the node is still running and there is a new frame available in
        `self.frame`, it publishes the frame as a compressed image message to the ROS topic `"image"` using
        the `self.publish_frame()` method. It then sleeps for a duration of time specified by `self.rate`
        before checking again. This ensures that frames are continuously published to the ROS network at a
        consistent rate.
        '''
        #rate = rospy.Rate(rospy.get_param("/camera/fps_publish"))
        cap = cv.VideoCapture(camera_id)
        self.set_camera(cap)
        self.print_camera_info(cap)

        while not rospy.is_shutdown():
            ret, frame = cap.read()

            if frame is None:
                continue

            frame[0, 0] = [self.camera_ids[camera_id]]*3
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            decoded_text, points, _ = self.qr_decoder.detectAndDecode(gray)
            if len(decoded_text)>0:
                #rospy.loginfo('RX QR code: %s', decoded_text)
                if len(points) > 0:
                    self.pub.publish(decoded_text)
            #rate.sleep()

    
if __name__ == "__main__":
    NODE_NAME = "realsense_node"
    node = Node()