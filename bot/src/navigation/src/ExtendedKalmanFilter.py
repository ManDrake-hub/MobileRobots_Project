#! /usr/bin/python3
import rospy
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class ExtendedKalmanFilter:
    def __init__(self):
        # Inizializza i parametri del filtro di Kalman
        self._pose = np.array((0.0, 0.0, 0.0))
        # Inizializza i publisher e i subscriber
        self.laser_sub = rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.ekf_callback)

    def get_rotation(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        return yaw
    
    def ekf_callback(self, msg):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        self._pose[2] = self.get_rotation(msg.pose.pose.orientation)
    
    def get_ekf_position(self):
        return self._pose