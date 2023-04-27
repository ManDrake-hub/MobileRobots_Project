#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from numpy.linalg import inv


class KalmanFilter:
    def __init__(self, update_step):
        #########################################
        # Init parameters and subscriber laser
        self.dt = update_step  
        self.F = np.array([[1, self.dt], [0, 1]])  # state-transition
        self.B = np.array([[self.dt**2 / 2], [self.dt]])  # control-input 
        self.H = np.array([[1, 0]])  # observation
        self.Q = np.array([[0.01, 0], [0, 0.01]])  # covariance of the process noise
        self.R = np.array([[0.1]])  # covariance of the observation noise
        self.P = np.array([[1, 0], [0, 1]])  # covariance of the estimation error
        self.x = np.array([[0], [0]])  # initial state
        self.distance = None
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
    
    #########################################
    # Utils
    def pose_and_velocity_to_x(self, pose, velocity):
        return np.array([[pose[0]], [velocity]])

    def x_to_pose(self, x):
        return np.array([x[0, 0], 0, 0])

    def laser_callback(self, msg):
        min_dist = msg.ranges[0]
        if self.distance is None:
            self.distance = min_dist
        self.z = np.array([[self.distance - min_dist]])  

    #########################################
    # Kalman Prediction and update
    # State and covariance of the estimation error prediction
    def kf_predict(self, u):
        x = np.dot(self.F, self.x) + np.dot(self.B, u)
        P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q 
        return x, P

    # State and covariance of the estimation error update
    def kf_update(self, X, P, Z, H, R):
        K =  np.dot(np.dot(P, H.T), inv(np.dot(np.dot(H, P), H.T) + R))  # Kalman gain
        X = X + np.dot(K, (Z - np.dot(H, X))) 
        P = P - np.dot((np.eye(2) - np.dot(K, H)), P)
        return (X, P)
    
    # Pose estimation
    def update_position(self, u):
        x_pred, P_pred = self.kf_predict(u)
        self.x, self.P = self.kf_update(x_pred, P_pred, self.z, self.H, self.R)

    #########################################
    # Getter
    def get_kf_position(self):
        return self.x_to_pose(self.x)