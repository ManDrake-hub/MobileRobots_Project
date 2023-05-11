#! /usr/bin/python3
import rospy
import csv
from std_msgs.msg import String, Int32MultiArray
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
        
        self.listener = tf.TransformListener()
        #self.pub_next_waypoint = rospy.Publisher('next_waypoint', Int32MultiArray, queue_size=10)

        # Set initial robot position and orientation
        self.robot_x = None
        self.robot_y = None
        self.robot_orientation = None
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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    # Calculate the distance between two points
    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Orientation required to reach a point
    def orientation_to_point(self, x, y):
        dx = x - self.robot_x
        dy = y - self.robot_y
        return math.atan2(dy, dx)

    # Function to execute a robot command and update its position/orientation
    def execute_command(self, command):
        if command == "straight on":
            self.robot_x += math.cos(self.robot_orientation)
            self.robot_y += math.sin(self.robot_orientation)
        elif command == "go back":
            self.robot_x -= math.cos(self.robot_orientation)
            self.robot_y -= math.sin(self.robot_orientation)
        elif command == "right":
            self.robot_orientation -= math.pi / 2  # 90 degrees
        elif command == "left":
            self.robot_orientation += math.pi / 2  # 90 degrees

    # Function to find the closest waypoint to the robot
    def find_closest_waypoint(self, command):
        closest_waypoint = None
        closest_distance = float('inf')
        waypoint_locations = [wp for wp in self.waypoints for i in range(len(wp)) 
                              if self.next_waypoint is None or wp[i] != self.next_waypoint[i] or 
                              (self.robot_x < self.next_waypoint[i] - self.thr and self.robot_x > self.next_waypoint[i] + self.thr)]
        for waypoint in waypoint_locations:
            if command == "right":
                if (waypoint[0] - self.robot_x) * math.sin(self.robot_orientation) - (waypoint[1] - self.robot_y) * math.cos(self.robot_orientation) < 0:
                    continue
            elif command == "left":
                if (waypoint[0] - self.robot_x) * math.sin(self.robot_orientation) + (waypoint[1] - self.robot_y) * math.cos(self.robot_orientation) < 0:
                    continue
            elif command == "straight on":
                if (waypoint[0] - self.robot_x) * math.cos(self.robot_orientation) + (waypoint[1] - self.robot_y) * math.sin(self.robot_orientation) < 0:
                    continue
            elif command == "go back":
                if (waypoint[0] - self.robot_x) * math.cos(self.robot_orientation) - (waypoint[1] - self.robot_y) * math.sin(self.robot_orientation) < 0:
                    continue
            d = self.distance(self.robot_x, self.robot_y, waypoint[0], waypoint[1])
            if d < closest_distance:
                closest_waypoint = waypoint
                closest_distance = d
        return closest_waypoint
    
    def navigate(self, command):
        self.get_robot_position()
        #### TO DO: COMMAND BASE
        #self.execute_command(command)
        self.next_waypoint = self.find_closest_waypoint(command)
        self.next_waypoint = self.next_waypoint + (self.orientation_to_point(self.next_waypoint[0], self.next_waypoint[1]),)
        print("Closest waypoint and orientation to reach it:", self.next_waypoint)
        return self.next_waypoint