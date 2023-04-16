#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Waypoint del robot
        self.waypoints = []
        self.current_waypoint = None

    def kf_predict(self, X, P, F, Q, B, U):
        X = np.dot(F, X) + np.dot(B, U) # predizione dello stato
        P = np.dot(F, np.dot(P, F.T)) + Q # predizione della covarianza dell'errore di stima
        return(X,P) 
        
    #LH: the Predictive probability (likelihood) of measurement
    def kf_update(self, X, P, Z, H, R):
        K =  np.dot(np.dot(P, H.T), inv(np.dot(np.dot(H, P), H.T) + R))  # calcolo del guadagno di Kalman
        X = X + np.dot(K, (Z - np.dot(H, X))) # correzione dello stato stimato
        P = P - np.dot((np.eye(2) - np.dot(K, H)), P)
        #LH = gauss_pdf(Y, IM, IS)
        return (X,P)

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint = self.waypoints[0]

    def laser_callback(self, msg):
        # Calcola la distanza minima dal sensore laser
        min_dist = min(msg.ranges)
        # Esegue il filtro di Kalman per stimare la posizione del robot
        x_pred, P_pred = self.kf_predict(self.x, self.P, self.F, self.Q, self.B, self.u)
        z = np.array([[min_dist]])  # misura della distanza dal sensore laser
        self.x, self.P = self.kf_update(x_pred, P_pred, z, self.H, self.R)

        # Controllo del Turtlebot utilizzando il filtro di Kalman
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # velocità lineare del robot
        cmd_vel.angular.z = -0.5 * self.x[0][0]  # velocità angolare proporzionale alla posizione stimata dal filtro di Kalman
        self.cmd_vel_pub.publish(cmd_vel)

    def odom_callback(self, msg):
        # Calcolo della posizione e dell'orientamento del robot a partire dalla posa fornita da ROS
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.pose = np.array([[x], [y], [yaw]])

        # Controllo dei waypoint
        if self.current_waypoint is not None:
            # Calcolo della distanza dal waypoint corrente
            dist = np.sqrt((self.current_waypoint[0] - x)**2 + (self.current_waypoint[1] - y)**2)
            
            # Se il robot è vicino al waypoint, passa al successivo
            if dist < 0.1:
                if len(self.waypoints) > 1:
                    self.waypoints = self.waypoints[1:]
                    self.current_waypoint = self.waypoints[0]
                else:
                    self.current_waypoint = None
            
            # Se il robot non ha ancora raggiunto l'ultimo waypoint, calcola l'errore di posizione
            if self.current_waypoint is not None:
                x_error = self.current_waypoint[0] - x
                y_error = self.current_waypoint[1] - y
                pos_error = np.array([[x_error], [y_error]])
                
                # Correzione dell'errore di posizione utilizzando il filtro di Kalman
                K_pos = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(np.dot(np.dot(self.C, self.P), self.C.T) + self.R)))
                self.pose = self.pose + np.dot(K_pos, (pos_error - np.dot(self.C, self.pose)))
                self.P = np.dot((np.eye(3) - np.dot(K_pos, self.C)), self.P)
                
                # Calcolo dell'angolo da seguire per raggiungere il waypoint
                target_angle = np.arctan2(y_error, x_error)
                
                # Correzione dell'errore di orientamento utilizzando il filtro di Kalman
                K_yaw = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(np.dot(np.dot(self.C, self.P), self.C.T) + self.R)))
                yaw_error = target_angle - yaw
                if yaw_error > np.pi:
                    yaw_error -= 2 * np.pi
                elif yaw_error < -np.pi:
                    yaw_error += 2 * np.pi
                self.pose[2][0] = yaw + np.dot(K_yaw, (yaw_error - np.dot(self.C, self.pose)))[0][0]
                self.P = np.dot((np.eye(3) - np.dot(K_yaw, self.C)), self.P)

        # Controllo del Turtlebot utilizzando il filtro di Kalman e la posizione corretta
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # velocità lineare del robot
        cmd_vel.angular.z = -0.5 * self.pose[0][0]  # velocità angolare proporzionale alla posizione stimata dal filtro di Kalman
        self.cmd_vel_pub.publish(cmd_vel)


if __name__ == 'main':
    rospy.init_node('kalman_filter')
    kf = KalmanFilter()
    kf.set_waypoints([
        np.array((0.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        np.array((2.0, 0.0, 0.0)),
        np.array((3.0, 0.0, 0.0)),
        np.array((4.0, 0.0, 0.0))
    ])
    rospy.spin()
