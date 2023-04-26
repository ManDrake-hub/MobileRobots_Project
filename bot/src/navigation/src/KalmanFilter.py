#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from numpy.linalg import inv


class KalmanFilter:
    def __init__(self, update_step):
        # Inizializza i parametri del filtro di Kalman
        self.dt = update_step  # intervallo di tempo tra le misure
        self.F = np.array([[1, self.dt], [0, 1]])  # matrice di stato
        self.B = np.array([[self.dt**2 / 2], [self.dt]])  # matrice di controllo
        self.H = np.array([[1, 0]])  # matrice di osservazione
        self.Q = np.array([[0.01, 0], [0, 0.01]])  # matrice di covarianza del rumore di processo
        self.R = np.array([[0.1]])  # matrice di covarianza del rumore di misura
        self.P = np.array([[1, 0], [0, 1]])  # matrice di covarianza dell'errore di stima
        self.x = np.array([[0], [0]])  # stato iniziale del sistema
        self.distance = None

        # Inizializza i publisher e i subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def kf_predict(self, u):
        x = np.dot(self.F, self.x) + np.dot(self.B, u) # predizione dello stato
        P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q # predizione della covarianza dell'errore di stima
        return x, P

    # LH: the Predictive probability (likelihood) of measurement
    def kf_update(self, X, P, Z, H, R):
        K =  np.dot(np.dot(P, H.T), inv(np.dot(np.dot(H, P), H.T) + R))  # calcolo del guadagno di Kalman
        X = X + np.dot(K, (Z - np.dot(H, X))) # correzione dello stato stimato
        P = P - np.dot((np.eye(2) - np.dot(K, H)), P)
        return (X, P)

    def pose_and_velocity_to_x(self, pose, velocity):
        return np.array([[pose[0]], [velocity]])

    def x_to_pose(self, x):
        return np.array([x[0, 0], 0, 0])

    def laser_callback(self, msg):
        min_dist = msg.ranges[0]
        if self.distance is None:
            self.distance = min_dist
        # Calcola la distanza minima dal sensore laser
        self.z = np.array([[self.distance - min_dist]])  # misura della distanza dal sensore laser

    def update_position(self, u):
        # Esegue il filtro di Kalman per stimare la posizione del robot
        x_pred, P_pred = self.kf_predict(u)
        self.x, self.P = self.kf_update(x_pred, P_pred, self.z, self.H, self.R)

    def get_kf_position(self):
        return self.x_to_pose(self.x)