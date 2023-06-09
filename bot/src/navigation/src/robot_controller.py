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
        self.listener = tf.TransformListener()
        # Set initial robot position and orientation
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.thr = 1.0
        self.orientation = None
        self.rot = None

        self.load_waypoints(waypoints_path)

    def load_waypoints(self, waypoints_path):
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))

    def clip_angle(self, angle):
        distances = [abs(angle - 0), abs(angle - 90), abs(angle + 90), abs(angle - 180)]
        min_distance = min(distances)
        if min_distance == distances[0]:
            return 0
        elif min_distance == distances[1]:
            return 90
        elif min_distance == distances[2]:
            return -90
        else:
            return 180
        
    # Get robot position 
    def get_robot_position(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            self.robot_z = self.clip_angle(math.degrees(rot[2]))
            self.actual_waypoint = (self.robot_x,self.robot_y)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def distance_waypoints(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return distance
    
    def normalize(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def get_angle_of_waypoints(self, reference, target):
        angle_degrees = math.degrees(math.atan2(target[1] - reference[1], target[0] - reference[0]))
        return self.normalize(angle_degrees)
    
    def get_angle_ranges(self, orientation):
        self.ranges = {'straight on': [-45, 0, 0, 45], 'go back': [135, 180, -180, -135],
                'right': [-135, -90, -90, -45], 'left': [45, 90, 90, 135]}
        for key, value in self.ranges.items():
            value[0] = self.normalize(value[0] + orientation)
            value[3] = self.normalize(value[3] + orientation)
            if value[0] == 135 and value[3] == -135:
                value[1] = 180
                value[2] = -180
            elif value[0] == -135 and value[3] == 135:
                value[1] = -180
                value[2] = 180
            else:
                value[1] = value[2] = (value[0] + value[3]) / 2
        return self.ranges

    def find_closest_waypoint(self, command, actual_waypoint):
        self.filter_waypoints = {"straight on": [None], "left": [None], "right": [None], "go back": [None]}

        #rospy.loginfo(f'actual waypoint {actual_waypoint}')
        if self.orientation is None:
            self.orientation = self.robot_z

        ranges = self.get_angle_ranges(self.orientation)

        waypoint_min = None
        min_distance = math.inf
        quat_orientation = None
        simple_rotate = None
        next_orientation = None

        for waypoint in self.waypoints:
            if (waypoint[0] - OFFSET <= actual_waypoint[0] <= waypoint[0] + OFFSET and
                    waypoint[1] - OFFSET <= actual_waypoint[1] <= waypoint[1] + OFFSET):
                continue

            angle_of_waypoints = self.get_angle_of_waypoints(actual_waypoint, waypoint)

            if (ranges["straight on"][0] <= angle_of_waypoints <= ranges["straight on"][1] or
                    ranges["straight on"][2] <= angle_of_waypoints <= ranges["straight on"][3]) and command == "straight on":
                self.filter_waypoints[command].append(waypoint)
                next_orientation = 0 + self.orientation
                simple_rotate = 0
            elif (ranges["go back"][0] <= angle_of_waypoints <= ranges["go back"][1] or
                ranges["go back"][2] <= angle_of_waypoints <= ranges["go back"][3]) and command == "go back":
                self.filter_waypoints[command].append(waypoint)
                next_orientation = 180 + self.orientation
                simple_rotate = 180
            elif (ranges["right"][0] <= angle_of_waypoints <= ranges["right"][1] or
                ranges["right"][2] <= angle_of_waypoints <= ranges["right"][3]) and command == "right":
                self.filter_waypoints[command].append(waypoint)
                next_orientation = -90 + self.orientation
                simple_rotate = -90
            elif (ranges["left"][0] <= angle_of_waypoints <= ranges["left"][1] or
                ranges["left"][2] <= angle_of_waypoints <= ranges["left"][3]) and command == "left":
                self.filter_waypoints[command].append(waypoint)
                next_orientation = 90 + self.orientation
                simple_rotate = 90

        for i in range(len(self.filter_waypoints[command])):
            if self.filter_waypoints[command][i] != None:
                waypoint_distance = self.distance_waypoints(actual_waypoint[0], actual_waypoint[1], self.filter_waypoints[command][i][0], self.filter_waypoints[command][i][1])
                if waypoint_distance < min_distance:
                    min_distance = waypoint_distance
                    waypoint_min = self.filter_waypoints[command][i]

        if len(self.filter_waypoints[command]) != 1:
            self.orientation = next_orientation
        quat_orientation = quaternion_from_euler(0,0,math.radians(next_orientation))
        rospy.loginfo(f'final orientation {self.orientation}')

        return waypoint_min, quat_orientation, simple_rotate

    def navigate(self, command, actual_waypoint):
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position()
        self.next_waypoint, orientation, angle = self.find_closest_waypoint(command, self.actual_waypoint)
        print(f"actual_waypoint: {self.actual_waypoint}, next_waypoint: {self.next_waypoint}")
        print("Closest waypoint and orientation to reach it:", self.next_waypoint, orientation)
        return self.next_waypoint, command,orientation,angle