#! /usr/bin/python3
import rospy
import csv
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist
import numpy as np 
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

class RobotController:
    def __init__(self, waypoints_path):
        self.waypoints = []
        # Read waypoints from CSV file
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))

        #self.pub_next_waypoint = rospy.Publisher('next_waypoint', Int32MultiArray, queue_size=10)
        self.listener = tf.TransformListener()
        # Set initial robot position and orientation
        self.robot_x = None
        self.robot_y = None
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.thr = 0.8
        self.orientation = None

    # Get robot position 
    def get_robot_position(self):
        try:
            # ------- Cerca il transform tra i frame di riferimento della posizione del robot e della mappa
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rot = euler_from_quaternion(rot)
            print('Robot position: {}'.format(trans))
            print('Robot orientation: {}'.format(rot))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            self.actual_waypoint = (self.robot_x,self.robot_y)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            
    def get_angle_of_waypoints(self, reference, target):
        # compute the two vectors formed by the points
        angle_degrees = math.degrees(math.atan2(target[1] - reference[1], target[0] - reference[0]))
        # adjust the angle to be between -180 and 180 degrees
        if angle_degrees > 180:
            angle_degrees -= 360
        elif angle_degrees < -180:
            angle_degrees += 360
        return angle_degrees

    def distance_waypoints(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance
    
    import matplotlib.pyplot as plt

    def plot_points_with_labels2(self,points, labels1,labels2):
        """
        Plotta dei punti nel piano con due etichette per ogni punto.
    
        Argomenti:
        - points: una lista di tuple (x, y) che rappresentano le coordinate dei punti
        -labels1: una lista di stringhe che rappresentano la prima etichetta dei punti
        - labels2: una lista di stringhe che rappresentano la seconda etichetta dei punti
    
        """
        x = []
        y = []
        for i in range(len(points)):
            x.append(points[i][0])
            y.append(points[i][1])
    
        plt.figure()
        plt.scatter(x, y)
    
        for i, (label1, label2) in enumerate(zip(labels1, labels2)):
            combined_label = str(label1) + "_" + str(label2)  # Concatena le due etichette separate con uno spazio
            plt.annotate(combined_label, (x[i], y[i]), textcoords="offset points", xytext=(0,10), ha='center')
    
        plt.show()
    
    def plot_points_with_labels3(self,points, labels1,labels2,labels3):
        """
        Plotta dei punti nel piano con due etichette per ogni punto.
    
        Argomenti:
        - points: una lista di tuple (x, y) che rappresentano le coordinate dei punti
        -labels1: una lista di stringhe che rappresentano la prima etichetta dei punti
        - labels2: una lista di stringhe che rappresentano la seconda etichetta dei punti
    
        """
        x = []
        y = []
        for i in range(len(points)):
            x.append(points[i][0])
            y.append(points[i][1])
    
        plt.figure()
        plt.scatter(x, y)
    
        for i, (label1, label2,label3) in enumerate(zip(labels1, labels2,labels3)):
            combined_label = str(label1) + "_" + str(label2)+str(labels3) # Concatena le due etichette separate con uno spazio
            plt.annotate(combined_label, (x[i], y[i]), textcoords="offset points", xytext=(0,10), ha='center')
    
        plt.show()
    
    import matplotlib.pyplot as plt

    def plot_points_with_labels(self,points, labels):
        """
        Plotta dei punti nel piano con le rispettive label.
    
        Argomenti:
        - points: una lista di tuple (x, y) che rappresentano le coordinate dei punti
        - labels: una lista di stringhe che rappresentano le label dei punti
    
        """
        x = [point[0] for point in points]
        y = [point[1] for point in points]
    
        plt.figure()
        plt.scatter(x, y)
    
        for i, label in enumerate(labels):
            text = str(x[i])+"_"+str(y[i])+"_"+str(label)
            plt.annotate(text, (x[i], y[i]), textcoords="offset points", xytext=(0,10), ha='center')
    
        plt.show()

    
    def find_closest_waypoint(self, command, actual_waypoint):
        waypoint_min = None
        self.filter_waypoints = {"straight_on": [None], "left": [None], "right": [None], "go_back":[None]}
        distances = []
        commands = []
        poss = []
        rospy.loginfo(f'actual waypoint {self.actual_waypoint}')
        orientamento = None
        rospy.loginfo(f'orientamento attuale del robot {self.orientation}')
        rospy.loginfo(f'comando attuale {command}')
        flag = None
        angle = None

        if self.orientation == None or self.orientation == 0:
            for i in range(len(self.waypoints)):
                distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
                commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

                if self.waypoints[i][0] == self.actual_waypoint[0] and self.waypoints[i][1] == self.actual_waypoint[1]:
                    continue
                if -45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45 and command == "straight_on": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    orientamento = quaternion_from_euler(0,0,0)
                    #rospy.loginfo(f'sono in questo if 1 1, orientamento {euler_from_quaternion(orientamento)}')
                    commands[-1] = commands[-1]+'*'
                    flag = 0
                elif (-180 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135 or 135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180) and command == "go_back":
                    self.filter_waypoints[command].append(self.waypoints[i])
                    orientamento = quaternion_from_euler(0,0,180)
                    #rospy.loginfo(f'sono in questo if 1 2, orientamento {euler_from_quaternion(orientamento)}')
                    commands[-1] = commands[-1]+'*'
                    flag = 180
                elif -135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45 and command == "right": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 1 3, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,-90)
                    commands[-1] = commands[-1]+'*'
                    flag = -90
                elif 45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135 and command == "left": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 1 4, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,90)
                    commands[-1] = commands[-1]+'*'
                    flag = 90
        elif self.orientation == -90:
            for i in range(len(self.waypoints)):
                distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
                commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

                if self.waypoints[i][0] == self.actual_waypoint[0] and self.waypoints[i][1] == self.actual_waypoint[1]:
                    continue
                if -135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45 and command == "straight_on": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 2 1, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,-90)
                    commands[-1] = commands[-1]+'*'
                    flag = -90
                if 45 <=  self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135 and command == 'go_back':
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 2 2, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,90)
                    commands[-1] = commands[-1]+'*'
                    flag = 90
                if (-180 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135 or 135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180) and command == "right": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    orientamento = quaternion_from_euler(0,0,180)
                    #rospy.loginfo(f'sono in questo if 2 3, orientamento {euler_from_quaternion(orientamento)}')
                    commands[-1] = commands[-1]+'*'
                    flag = 180
                if -45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45 and command == "left": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 2 4, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,0)
                    commands[-1] = commands[-1]+'*'
                    flag = 0
        elif self.orientation == 90:
            for i in range(len(self.waypoints)):
                distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
                commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

                if self.waypoints[i][0] == self.actual_waypoint[0] and self.waypoints[i][1] == self.actual_waypoint[1]:
                    continue
                if 45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135 and command == "straight_on": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 3 1, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,90)
                    commands[-1] = commands[-1]+'*'
                    flag = 90
                if -135 <=  self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45 and command == 'go_back':
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 3 2, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,-90)
                    commands[-1] = commands[-1]+'*'
                    flag = -90
                if -45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45 and command == "right": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    orientamento = quaternion_from_euler(0,0,0)
                    #rospy.loginfo(f'sono in questo if 3 3, orientamento {euler_from_quaternion(orientamento)}')
                    commands[-1] = commands[-1]+'*'
                    flag = 0
                if (-180 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135 or 135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180)  and command == "left": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    orientamento = quaternion_from_euler(0,0,180)
                    #rospy.loginfo(f'sono in questo if 3 4, orientamento {euler_from_quaternion(orientamento)}')
                    commands[-1] = commands[-1]+'*'
                    flag = 180
        elif self.orientation == 180:
            for i in range(len(self.waypoints)):
                distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
                commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

                if self.waypoints[i][0] == self.actual_waypoint[0] and self.waypoints[i][1] == self.actual_waypoint[1]:
                    continue
                if (-180 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135 or 135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180) and command == "straight_on": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 4 1, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,180)
                    commands[-1] = command[-1]+'*'
                    flag = 180
                if -45 <=  self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45 and command == 'go_back':
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 4 2, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,0)
                    commands[-1] = commands[-1]+'*'
                    flag = 0
                if 45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135 and command == "right": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 4 3, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,90)
                    commands[-1] = commands[-1]+'*'
                    flag = 90
                if -135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45 and command == "left": 
                    self.filter_waypoints[command].append(self.waypoints[i])
                    #rospy.loginfo(f'sono in questo if 4 4, orientamento {euler_from_quaternion(orientamento)}')
                    orientamento = quaternion_from_euler(0,0,-90)
                    commands[-1] = commands[-1]+'*'
                    flag = -90

        self.plot_points_with_labels2(points=self.waypoints,labels1=distances,labels2=commands)
        min = float('inf')
        for i in range(len(self.filter_waypoints[command])):
            if self.filter_waypoints[command][i] != None:
                distance = self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.filter_waypoints[command][i][0],self.filter_waypoints[command][i][1])
                if distance <min:
                    min = distance
                    waypoint_min = self.filter_waypoints[command][i]
        self.orientation = flag
        rospy.loginfo(f'orientamento finale {self.orientation}')
        if waypoint_min is not None:
            angle = self.get_angle_of_waypoints(actual_waypoint,waypoint_min)
        return waypoint_min,orientamento,angle
    

    def navigate(self, command, actual_waypoint):
        #self.execute_command(command)
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position()
        self.next_waypoint, orientation,angle = self.find_closest_waypoint(command, self.actual_waypoint)
        print(f"actual_waypoint: {self.actual_waypoint}, next_waypoint: {self.next_waypoint}")
        print("Closest waypoint and orientation to reach it:", self.next_waypoint)
        return self.next_waypoint, command,orientation,angle
    