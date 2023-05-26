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
import os

OFFSET = 0.5
class RobotController:
    def __init__(self, waypoints_path):
        self.waypoints = []
        # Read waypoints from CSV file
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))

        self.listener = tf.TransformListener()
        # Set initial robot position and orientation
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
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
            self.robot_z = rot
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
    
    def normalize(self,angle):
        new_angle = angle
        if angle > 180:
            diff = angle -180
            new_angle = -180 +diff
        elif angle < -180:
            diff = angle + 180
            new_angle = 180 + diff
        return new_angle
    
    def get_angle_ranges(self,orientation):
        ranges = {'straight_on': [-45,0,0,45],'go_back':[135,180,-180,-135],'right':[-135,-90,-90,-45],'left':[45,90,90,135]}
        commands = ['straight_on','go_back','right','left']
        for i in range(len(commands)):
            ranges[commands[i]][0] = self.normalize(ranges[commands[i]][0] + orientation)
            ranges[commands[i]][3] = self.normalize(ranges[commands[i]][3] + orientation)
            if ranges[commands[i]][0] == 135 and ranges[commands[i]][3] == -135:
                ranges[commands[i]][1] = 180
                ranges[commands[i]][2] = -180
            elif ranges[commands[i]][0] == -135 and ranges[commands[i]][3] == 135:
                ranges[commands[i]][1] = -180
                ranges[commands[i]][2] = 180
            else:
                ranges[commands[i]][1] = ranges[commands[i]][2] = (ranges[commands[i]][0] + ranges[commands[i]][3])/2
        return ranges
    
    def clip_angle(self,angle):
        dist_0 = abs(angle - 0)
        dist_90 = abs(angle - 90)
        dist_minus90 = abs(angle + 90)
        dist_180 = abs(angle - 180)

        minima_distanza = min(dist_0, dist_90, dist_minus90, dist_180)

        if minima_distanza == dist_0:
            return 0
        elif minima_distanza == dist_90:
            return 90
        elif minima_distanza == dist_minus90:
            return -90
        else:
            return 180

    def find_closest_waypoint(self, command, actual_waypoint):
        waypoint_min = None
        self.filter_waypoints = {"straight_on": [None], "left": [None], "right": [None], "go_back":[None]}
        distances = []
        commands = []
        poss = []
        rospy.loginfo(f'actual waypoint {self.actual_waypoint}')
        orientamento = None
        rospy.loginfo(f'orientamento attuale del robot {self.robot_z}')
        rospy.loginfo(f'comando attuale {command}')
        flag = None
        angle = None       
        #if self.orientation == None or self.orientation == 0:
        for i in range(len(self.waypoints)):
            distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
            commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

            if self.waypoints[i][0] - OFFSET <= self.actual_waypoint[0] <= self.waypoints[i][0] + OFFSET and self.waypoints[i][1] - OFFSET <= self.actual_waypoint[1] <= self.waypoints[i][1] + OFFSET :
                continue
            #TODO-to fix. insert real pose

            if self.orientation == None:
                self.orientation = math.degrees(self.robot_z[2])
                self.orientation = self.clip_angle(self.orientation)

            ranges = self.get_angle_ranges(self.orientation)
            print(f'RANGES : {ranges}')

            if (ranges["straight_on"][0] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["straight_on"][1] or ranges["straight_on"][2] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["straight_on"][3]) and command == "straight_on": 
                self.filter_waypoints[command].append(self.waypoints[i])
                flag2 = 0+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                #rospy.loginfo(f'sono in questo if 1 1, orientamento {euler_from_quaternion(orientamento)}')
                commands[-1] = commands[-1]+'*'
                flag = 0
                print(f"flag {flag}")
            elif (ranges["go_back"][0] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["go_back"][1] or ranges["go_back"][2] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["go_back"][3]) and command == "go_back":
                self.filter_waypoints[command].append(self.waypoints[i])
                flag2 = 180+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                #rospy.loginfo(f'sono in questo if 1 2, orientamento {euler_from_quaternion(orientamento)}')
                commands[-1] = commands[-1]+'*'
                flag = 180
                print(f"flag {flag}")
            elif (ranges["right"][0] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["right"][1] or ranges["right"][2] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["right"][3]) and command == "right": 
                self.filter_waypoints[command].append(self.waypoints[i])
                #rospy.loginfo(f'sono in questo if 1 3, orientamento {euler_from_quaternion(orientamento)}')
                flag2 = -90+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                commands[-1] = commands[-1]+'*'
                flag = -90
                print(f"flag {flag}")
            elif (ranges["left"][0] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["left"][1] or ranges["left"][2] <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= ranges["left"][3]) and command == "left": 
                self.filter_waypoints[command].append(self.waypoints[i])
                #rospy.loginfo(f'sono in questo if 1 4, orientamento {euler_from_quaternion(orientamento)}')
                flag2 = 90+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                commands[-1] = commands[-1]+'*'
                flag = 90
                print(f"flag {flag}")

        #self.plot_points_with_labels2(points=self.waypoints,labels1=distances,labels2=commands)
        min = float('inf')
        for i in range(len(self.filter_waypoints[command])):
            if self.filter_waypoints[command][i] != None:
                distance = self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.filter_waypoints[command][i][0],self.filter_waypoints[command][i][1])
                if distance <min:
                    min = distance
                    waypoint_min = self.filter_waypoints[command][i]
                
        if len(self.filter_waypoints[command])!=1:
            self.orientation = flag2
        
        rospy.loginfo(f'orientamento finale {self.orientation}')
        #if waypoint_min is not None:
        #    angle = self.get_angle_of_waypoints(actual_waypoint,waypoint_min)
        return waypoint_min,orientamento,flag #CHEEEEECK     

    """def find_closest_waypoint(self, command, actual_waypoint):
        waypoint_min = None
        self.filter_waypoints = {"straight_on": [None], "left": [None], "right": [None], "go_back":[None]}
        distances = []
        commands = []
        poss = []
        rospy.loginfo(f'actual waypoint {self.actual_waypoint}')
        orientamento = None
        rospy.loginfo(f'orientamento attuale del robot {self.robot_z}')
        rospy.loginfo(f'comando attuale {command}')
        flag = None
        angle = None
        
        #if self.orientation == None or self.orientation == 0:
        for i in range(len(self.waypoints)):
            distances.append(self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.waypoints[i][0],self.waypoints[i][1]))
            commands.append(str(self.waypoints[i][0])+"_"+str(self.waypoints[i][1]))

            if self.waypoints[i][0] - OFFSET <= self.actual_waypoint[0] <= self.waypoints[i][0] + OFFSET and self.waypoints[i][1] - OFFSET <= self.actual_waypoint[1] <= self.waypoints[i][1] + OFFSET :
                continue
            #TODO-to fix. insert real pose

            if self.orientation == None:
                self.orientation = math.degrees(self.robot_z[2])
            if -45+self.orientation <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45+self.orientation and command == "straight_on": 
                self.filter_waypoints[command].append(self.waypoints[i])
                flag2 = 0+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                #rospy.loginfo(f'sono in questo if 1 1, orientamento {euler_from_quaternion(orientamento)}')
                commands[-1] = commands[-1]+'*'
                flag = 0
                print(f"flag {flag}")
            elif (-180+self.orientation <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135+self.orientation or 135+self.orientation <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180+self.orientation) and command == "go_back":
                self.filter_waypoints[command].append(self.waypoints[i])
                flag2 = 180+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                #rospy.loginfo(f'sono in questo if 1 2, orientamento {euler_from_quaternion(orientamento)}')
                commands[-1] = commands[-1]+'*'
                flag = 180
                print(f"flag {flag}")
            elif -135+self.orientation <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45+self.orientation and command == "right": 
                self.filter_waypoints[command].append(self.waypoints[i])
                #rospy.loginfo(f'sono in questo if 1 3, orientamento {euler_from_quaternion(orientamento)}')
                flag2 = -90+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                commands[-1] = commands[-1]+'*'
                flag = -90
                print(f"flag {flag}")
            elif 45+self.orientation <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135+self.orientation and command == "left": 
                self.filter_waypoints[command].append(self.waypoints[i])
                #rospy.loginfo(f'sono in questo if 1 4, orientamento {euler_from_quaternion(orientamento)}')
                flag2 = 90+self.orientation
                orientamento = quaternion_from_euler(0,0,math.radians(flag2))
                commands[-1] = commands[-1]+'*'
                flag = 90
                print(f"flag {flag}")

        #self.plot_points_with_labels2(points=self.waypoints,labels1=distances,labels2=commands)
        min = float('inf')
        for i in range(len(self.filter_waypoints[command])):
            if self.filter_waypoints[command][i] != None:
                distance = self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.filter_waypoints[command][i][0],self.filter_waypoints[command][i][1])
                if distance <min:
                    min = distance
                    waypoint_min = self.filter_waypoints[command][i]
                
        if len(self.filter_waypoints[command])!=1:
            self.orientation = flag2
        
        rospy.loginfo(f'orientamento finale {self.orientation}')
        #if waypoint_min is not None:
        #    angle = self.get_angle_of_waypoints(actual_waypoint,waypoint_min)
        return waypoint_min,orientamento,flag #CHEEEEECK"""
    

    def navigate(self, command, actual_waypoint):
        #self.execute_command(command)
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position()
        self.next_waypoint, orientation,angle = self.find_closest_waypoint(command, self.actual_waypoint)
        print(f"actual_waypoint: {self.actual_waypoint}, next_waypoint: {self.next_waypoint}")
        print("Closest waypoint and orientation to reach it:", self.next_waypoint, orientation)
        return self.next_waypoint, command,orientation,angle
    
