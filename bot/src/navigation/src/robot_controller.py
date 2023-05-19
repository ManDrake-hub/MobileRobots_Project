#! /usr/bin/python3
import rospy
import csv
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Twist
import numpy as np 
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
    
    def find_closest_waypoint (self, command, actual_waypoint):
        waypoint_min = ''
        self.filter_waypoints = {"straight_on": [None], "left": [None], "right": [None], "go_back":[None]}
        for i in range(len(self.waypoints)):
            if -45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 45 and command == "straight_on": 
                self.filter_waypoints[command].append(self.waypoints[i])
            if -180 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -135 or 135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 180 and command == "go_back": 
                self.filter_waypoints[command].append(self.waypoints[i])
            if 45 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= 135 and command == "right": 
                self.filter_waypoints[command].append(self.waypoints[i])
            if -135 <= self.get_angle_of_waypoints(actual_waypoint,self.waypoints[i]) <= -45 and command == "left": 
                self.filter_waypoints[command].append(self.waypoints[i])
        min = float('inf')
        for i in range(len(self.filter_waypoints[command])):
            if self.filter_waypoints[command][i] != None:
                distance = self.distance_waypoints(actual_waypoint[0],actual_waypoint[1],self.filter_waypoints[command][i][0],self.filter_waypoints[command][i][1])
                if distance <= min:
                    min = distance
                    waypoint_min = self.filter_waypoints[command][i]
        return waypoint_min
        

    def navigate(self, command, actual_waypoint):
        #self.execute_command(command)
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position()
        self.next_waypoint = self.find_closest_waypoint(command, self.actual_waypoint)
        print(f"actual_waypoint: {self.actual_waypoint}, next_waypoint: {self.next_waypoint}")
        print("Closest waypoint and orientation to reach it:", self.next_waypoint)
        return self.next_waypoint, command
    