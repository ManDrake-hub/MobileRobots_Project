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
        self.robot_orientation = None
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
            self.robot_orientation = rot[2]
            self.actual_waypoint = (self.robot_x,self.robot_y,self.robot_orientation)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
   
    # Calculate the distance between two points
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Orientation required to reach a point
    def orientation_to_point(self, x, y, actual_waypoint):
        dx = x - actual_waypoint[0]
        dy = y - actual_waypoint[1]
        return math.atan2(dy, dx)

    # Function to execute a robot command and update its position/orientation
    def execute_command(self, command, actual_waypoint):
        if command == "straight on":
            actual_waypoint[0] += math.cos(actual_waypoint[2])
            actual_waypoint[1] += math.sin(actual_waypoint[2])
        elif command == "go back":
            actual_waypoint[0] -= math.cos(actual_waypoint[2])
            actual_waypoint[1] -= math.sin(actual_waypoint[2])
        elif command == "right":
            actual_waypoint[2] -= math.pi / 2  # 90 degrees
        elif command == "left":
            actual_waypoint[2] += math.pi / 2  # 90 degrees

    # Function to find the closest waypoint to the robot
    def find_closest_waypoint(self, command, actual_waypoint):
        closest_waypoint = None
        closest_distance = float('inf')
        waypoint_locations = [wp for wp in self.waypoints for i in range(len(wp)) 
                              if wp[i] != self.actual_waypoint[i]]
        for waypoint in waypoint_locations:
            if command == "right":
                if (waypoint[0] - actual_waypoint[0]) * math.sin(actual_waypoint[2]) - (waypoint[1] - actual_waypoint[1]) * math.cos(actual_waypoint[2]) < 0:
                    continue
            elif command == "left":
                if (waypoint[0] - actual_waypoint[0]) * math.sin(actual_waypoint[2]) + (waypoint[1] - actual_waypoint[1]) * math.cos(actual_waypoint[2]) < 0:
                    continue
            elif command == "straight on":
                if (waypoint[0] - actual_waypoint[0]) * math.cos(actual_waypoint[2]) + (waypoint[1] - actual_waypoint[1]) * math.sin(actual_waypoint[2]) < 0:
                    continue
            elif command == "go back":
                if (waypoint[0] - actual_waypoint[0]) * math.cos(actual_waypoint[2]) - (waypoint[1] - actual_waypoint[1]) * math.sin(actual_waypoint[2]) < 0:
                    continue
            d = self.distance(actual_waypoint[0], actual_waypoint[1], waypoint[0], waypoint[1])
            if d < closest_distance:
                closest_waypoint = waypoint
                closest_distance = d
        return closest_waypoint
    
    def navigate(self, command, actual_waypoint):
        #### TO DO: COMMAND BASE
        #self.execute_command(command)
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position()
        self.next_waypoint = self.find_closest_waypoint(command, self.actual_waypoint)
        self.next_waypoint = self.next_waypoint + (self.orientation_to_point(self.next_waypoint[0], self.next_waypoint[1], self.actual_waypoint),)
        print("Closest waypoint and orientation to reach it:", self.next_waypoint)
        return self.next_waypoint
    