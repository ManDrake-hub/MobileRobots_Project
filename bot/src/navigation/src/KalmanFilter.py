#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from numpy.linalg import inv


class KalmanFilter:
    def __init__(self):
        # Inizializza i parametri del filtro di Kalman
        self.dt = 0.1  # intervallo di tempo tra le misure
        self.F = np.array([[1, self.dt], [0, 1]])  # matrice di stato
        self.B = np.array([[self.dt**2 / 2], [self.dt]])  # matrice di controllo
        self.H = np.array([[1, 0]])  # matrice di osservazione
        self.Q = np.array([[0.01, 0], [0, 0.01]])  # matrice di covarianza del rumore di processo
        self.R = np.array([[0.1]])  # matrice di covarianza del rumore di misura
        self.P = np.array([[1, 0], [0, 1]])  # matrice di covarianza dell'errore di stima
        self.x = np.array([[0], [0]])  # stato iniziale del sistema
        self.u = np.array([[0]])  # input di controllo iniziale

        # Inizializza i publisher e i subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def kf_predict(self, P, F, Q):
        P = np.dot(F, np.dot(P, F.T)) + Q # predizione della covarianza dell'errore di stima
        return P

    #LH: the Predictive probability (likelihood) of measurement
    def kf_update(self, X, P, Z, H, R):
        K =  np.dot(np.dot(P, H.T), inv(np.dot(np.dot(H, P), H.T) + R))  # calcolo del guadagno di Kalman
        X = X + np.dot(K, (Z - np.dot(H, X))) # correzione dello stato stimato
        P = P - np.dot((np.eye(2) - np.dot(K, H)), P)
        return (X, P)

    def pose_and_velocity_to_x(self, pose, velocity):
        return np.array([[pose[0]], [velocity]])

    def x_to_pose(self, x, pose):
        pose[0] = x[0, 0]
        return pose

    def laser_callback(self, msg):
        # Calcola la distanza minima dal sensore laser
        min_dist = msg.ranges[0]
        self.z = np.array([[min_dist]])  # misura della distanza dal sensore laser

    def get_kf_position(self, pose, velocity):
        # Esegue il filtro di Kalman per stimare la posizione del robot
        x = self.pose_and_velocity_to_x(pose, velocity)
        P_pred = self.kf_predict(self.P, self.F, self.Q)
        x, self.P = self.kf_update(x, P_pred, self.z, self.H, self.R)
        return self.x_to_pose(x, pose)