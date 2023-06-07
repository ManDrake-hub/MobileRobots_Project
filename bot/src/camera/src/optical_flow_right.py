#!/usr/bin/env python3
import rospy
from pyzbar.pyzbar import ZBarSymbol, ZBarConfig, decode
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray, Bool
from optparse import OptionParser

def estimate_movement(frame):

    # Convert the frame to grayscale
    prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Set up parameters for the optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create an array to hold the tracked points
    prev_pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    vertical_threshold = 10

    while True:
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **lk_params)

        if next_pts is not None:
            # Select good points
            good_old = prev_pts[status == 1]
            good_new = next_pts[status == 1]

            if len(good_new) == 0:
                prev_pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
                continue

            # Draw the movement vectors
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel().astype(int)
                c, d = old.ravel().astype(int)
                frame = cv2.line(frame, (a, b), (c, d), (0, 255, 0), 2)
                frame = cv2.circle(frame, (a, b), 5, (0, 255, 0), -1)
            
            # Check for upward and downward vertical movement
            downward_movement = any(old[1] - new[1] > vertical_threshold for new, old in zip(good_new, good_old))
            upward_movement = any(new[1] - old[1] > vertical_threshold for new, old in zip(good_new, good_old))

            if upward_movement:
                print("Vertical upward movement of points detected!")
                pub_upward.publish(True)
            if downward_movement:
                print("Vertical downward movement of points detected!")
                pub_downward.publish(True)
        else:
            prev_pts = cv2.goodFeaturesToTrack(prev_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        # Display the resulting frame
        cv2.imshow("Webcam Movement Estimation", frame)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # Set the current frame and points as the previous frame and points for the next iteration
        prev_gray = gray.copy()
        prev_pts = good_new.reshape(-1, 1, 2)

    cv2.imshow('rx', frame)
    cv2.waitKey(1)
    
def camera_image_rx_callback(image_msg):
    estimate_movement(image_msg)

if __name__ == '__main__':
    rospy.init_node('optical_flow_rx', anonymous=True)
    # REALITY
    sub_left = rospy.Subscriber('camera/rx/image', CompressedImage, camera_image_rx_callback)
    pub_upward = rospy.Publisher('upward', Bool, queue_size=1)
    pub_downward = rospy.Publisher('downward', Bool, queue_size=1)
    rospy.sleep(0.5)
    rospy.spin()