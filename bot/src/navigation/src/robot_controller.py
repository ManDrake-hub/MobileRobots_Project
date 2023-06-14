#! /usr/bin/python3
import rospy
import csv
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

OFFSET = 1.0

class RobotController:
    def __init__(self, waypoints_path):
        self.waypoints = []  # List to store the waypoints
        self.listener = tf.TransformListener()  # Listener to obtain transformations
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.load_waypoints(waypoints_path)  # Load waypoints from a CSV file

    def load_waypoints(self, waypoints_path):
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))  # Append the coordinates to the waypoints list

    def clip_angle(self, angle):
        angle_ranges = [0, 90, -90, 180, -180]  # Define angle ranges
        min_distance = min([abs(angle - a) for a in angle_ranges])  # Find the minimum distance to an angle range
        min_index = [abs(angle - a) for a in angle_ranges].index(min_distance)  # Find the index of the minimum distance
        return angle_ranges[min_index]  # Return the closest angle from the angle ranges

    # Get robot position
    def get_robot_position(self, flag):
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            if trans is None or rot is None:
                print("Was None")
                return

            self.robot_x, self.robot_y = trans[0], trans[1]  # Store the robot's x and y coordinates
            if flag:
                self.robot_z = self.clip_angle(math.degrees(euler_from_quaternion(rot)[2]))  # Convert and clip the robot's orientation angle
            else:
                self.robot_z = rot # Convert and clip the robot's orientation angle
            print(self.robot_z)
            self.actual_waypoint = (self.robot_x, self.robot_y)  # Set the actual waypoint as the robot's current position
        except Exception as e:
            rospy.sleep(0.25)
            self.get_robot_position(flag)

    def distance_waypoints(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # Calculate the distance between two waypoints
        return distance

    def normalize(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def get_angle_of_waypoints(self, reference, target):
        angle_radians = math.atan2(target[1] - reference[1], target[0] - reference[0])  # Calculate the angle between two waypoints
        angle_degrees = math.degrees(angle_radians)  # Convert the angle to degrees
        return self.normalize(angle_degrees)  # Normalize the angle

    def get_angle_ranges(self, orientation):
        ranges = {
            'straight on': [-45, 0, 0, 45],
            'go back': [135, 180, -180, -135],
            'right': [-135, -90, -90, -45],
            'left': [45, 90, 90, 135]
        }  # Define angle ranges for different commands
        for key, value in ranges.items():
            value[0] = self.normalize(value[0] + orientation)  # Adjust the angle ranges based on the current orientation
            value[3] = self.normalize(value[3] + orientation)
            if value[0] == 135 and value[3] == -135:
                value[1] = 180
                value[2] = -180
            elif value[0] == -135 and value[3] == 135:
                value[1] = -180
                value[2] = 180
            else:
                value[1] = value[2] = (value[0] + value[3]) / 2  # Calculate the mid-angle for the ranges
        return ranges

    def find_closest_waypoint(self, command):
        filter_waypoints = {"straight on": [None], "left": [None], "right": [None], "go back": [None]}
        ranges = self.get_angle_ranges(self.robot_z)  # Get the angle ranges based on the current orientation

        waypoint_min = None
        min_distance = math.inf
        simple_rotate = None
        next_quat_orientation = None
        next_orientation = None

        for waypoint in self.waypoints:
            if (waypoint[0] - OFFSET <= self.actual_waypoint[0] <= waypoint[0] + OFFSET and
                    waypoint[1] - OFFSET <= self.actual_waypoint[1] <= waypoint[1] + OFFSET):
                continue  # Skip waypoints within a certain offset range of the actual waypoint

            angle_of_waypoints = self.get_angle_of_waypoints(self.actual_waypoint, waypoint)

            if (ranges["straight on"][0] <= angle_of_waypoints <= ranges["straight on"][1] or
                    ranges["straight on"][2] <= angle_of_waypoints <= ranges["straight on"][3]) and command == "straight on":
                filter_waypoints[command].append(waypoint)
                next_orientation = 0 + self.robot_z
                simple_rotate = 0
            elif (ranges["go back"][0] <= angle_of_waypoints <= ranges["go back"][1] or
                  ranges["go back"][2] <= angle_of_waypoints <= ranges["go back"][3]) and command == "go back":
                filter_waypoints[command].append(waypoint)
                next_orientation = 180 + self.robot_z
                simple_rotate = 180
            elif (ranges["right"][0] <= angle_of_waypoints <= ranges["right"][1] or
                  ranges["right"][2] <= angle_of_waypoints <= ranges["right"][3]) and command == "right":
                filter_waypoints[command].append(waypoint)
                next_orientation = -90 + self.robot_z
                simple_rotate = -90
            elif (ranges["left"][0] <= angle_of_waypoints <= ranges["left"][1] or
                  ranges["left"][2] <= angle_of_waypoints <= ranges["left"][3]) and command == "left":
                filter_waypoints[command].append(waypoint)
                next_orientation = 90 + self.robot_z
                simple_rotate = 90

        for i in range(len(filter_waypoints[command])):
            if filter_waypoints[command][i] is not None:
                waypoint_distance = self.distance_waypoints(self.actual_waypoint[0], self.actual_waypoint[1],
                                                            filter_waypoints[command][i][0],
                                                            filter_waypoints[command][i][1])
                if waypoint_distance < min_distance:
                    min_distance = waypoint_distance
                    waypoint_min = filter_waypoints[command][i]

        actual_orientation = self.robot_z
        if len(filter_waypoints[command]) != 1:
            self.robot_z = next_orientation  # Update the orientation if multiple waypoints are found
        next_quat_orientation = quaternion_from_euler(0, 0, math.radians(self.robot_z))
        rospy.loginfo(f'actual orientation goal {actual_orientation}, final orientation goal {self.robot_z}')  # Log the final orientation
        return waypoint_min, next_quat_orientation, simple_rotate, actual_orientation

    def navigate(self, command, actual_waypoint):
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position(True)  # Obtain the robot's current position
        self.next_waypoint, next_orientation, rotate_angle, actual_orientation = self.find_closest_waypoint(command)
        print(f"actual waypoint: {self.actual_waypoint}, next waypoint: {self.next_waypoint}")
        return self.next_waypoint, next_orientation, rotate_angle, actual_orientation
