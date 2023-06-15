#! /usr/bin/python3
import rospy
import csv
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

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
        '''
        Loads the waypoints from a CSV file.

        Args:
            waypoints_path: The path to the CSV file containing the waypoints.
        '''
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))  # Append the coordinates to the waypoints list

    def clip_angle(self, angle):
        '''
        Clips the angle to the closest angle range. The angle ranges reflect the robot position 

        Args:
            angle: The angle to be clipped.

        Returns:
            The closest angle from the angle ranges.
        '''
        angle_ranges = [0, 90, -90, 180, -180]  
        min_distance = min([abs(angle - a) for a in angle_ranges])  
        min_index = [abs(angle - a) for a in angle_ranges].index(min_distance)  
        return angle_ranges[min_index] 

    def get_robot_position(self, flag):
        '''
        Obtains the robot's current position.

        Args:
            flag: A flag indicating whether to convert and clip the robot's orientation angle.
        '''
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            if trans is None or rot is None:
                print("Was None")
                return

            self.robot_x, self.robot_y = trans[0], trans[1]  # Store the robot's x and y coordinates
            if flag:
                self.robot_z = self.clip_angle(math.degrees(euler_from_quaternion(rot)[2]))  # Convert and clip the robot's orientation angle
            else:
                self.robot_z = rot
            print(self.robot_z)
            self.actual_waypoint = (self.robot_x, self.robot_y)  # Set the actual waypoint as the robot's current position
        except Exception as e:
            rospy.sleep(0.25)
            self.get_robot_position(flag)

    def distance_waypoints(self, x1, y1, x2, y2):
        '''
        Calculates the distance between two waypoints.

        Args:
            x1: X-coordinate of the first waypoint.
            y1: Y-coordinate of the first waypoint.
            x2: X-coordinate of the second waypoint.
            y2: Y-coordinate of the second waypoint.

        Returns:
            The distance between the two waypoints.
        '''
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) 
        return distance

    def normalize(self, angle):
        '''
        Normalizes the angle to the range [-180, 180].

        Args:
            angle: The angle to be normalized.

        Returns:
            The normalized angle.
        '''
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def get_angle_of_waypoints(self, reference, target):
        '''
        Calculates the angle between two waypoints.

        Args:
            reference: The coordinates of the reference waypoint.
            target: The coordinates of the target waypoint.

        Returns:
            The angle between the two waypoints.
        '''
        angle_radians = math.atan2(target[1] - reference[1], target[0] - reference[0])  
        angle_degrees = math.degrees(angle_radians)  
        return self.normalize(angle_degrees)  # Normalize the angle

    def get_angle_ranges(self, orientation):
        '''
        Calculates the angle ranges for different commands based on the current orientation.

        Args:
            orientation: The current orientation angle.

        Returns:
            dict: A dictionary containing the angle ranges for different commands.
        '''
        ranges = {
            'straight on': [-45, 0, 0, 45],
            'go back': [135, 180, -180, -135],
            'right': [-135, -90, -90, -45],
            'left': [45, 90, 90, 135]
        }  # Define angle ranges for different commands
        for key, value in ranges.items():
            value[0] = self.normalize(value[0] + orientation)  
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
    
    def plot_waypoints(self,points, labels1,labels2):
        '''
        Plot the points in the plane with two labels for each point.
    
        Subjects:
        - points: a list of tuples (x, y) representing the coordinates of the points
        -labels1: a list of strings representing the first point label
        - labels2: a list of strings representing the second point label
    
        '''
        x = []
        y = []
        for i in range(len(points)):
            x.append(points[i][0])
            y.append(points[i][1])
    
        plt.figure()
        plt.scatter(x, y)
    
        for i, (label1, label2) in enumerate(zip(labels1, labels2)):
            combined_label = str(round(label1,1)) + "_" + str(label2) 
            plt.annotate(combined_label, (x[i], y[i]), textcoords="offset points", xytext=(0,10), ha='center')
    
        plt.show()

    def find_closest_waypoint(self, command):
        '''
        Finds the closest waypoint based on the command and the current robot position.

        Args:
            command: The command indicating the desired direction.

        Returns:
            The coordinates of the closest waypoint.
            The next quaternion orientation.
            The rotation angle for simple rotation.
            The actual orientation before adjusting for multiple waypoints.
        '''
        filter_waypoints = {"straight on": [None], "left": [None], "right": [None], "go back": [None]}
        ranges = self.get_angle_ranges(self.robot_z)  # Get the angle ranges based on the current orientation
        distances = []
        commands = []

        waypoint_min = None
        min_distance = math.inf
        simple_rotate = None
        next_quat_orientation = None
        next_orientation = None
        # Find the closest waypoint to the current robot position
        for waypoint in self.waypoints:
            waypoint_distance = self.distance_waypoints(waypoint[0], waypoint[1],
                                                            self.actual_waypoint[0],
                                                            self.actual_waypoint[1])
            if waypoint_distance < min_distance:
                min_distance = waypoint_distance
                waypoint_min = (waypoint[0],waypoint[1])
        min_distance = math.inf
        # Filter waypoints based on the command and angle ranges
        for waypoint in self.waypoints:
            if waypoint == waypoint_min:
                continue  # Skip waypoint near the actual waypoint
            distances.append(self.distance_waypoints(self.actual_waypoint[0],self.actual_waypoint[1],waypoint[0], waypoint[1]))
            commands.append(str(waypoint[0])+"_"+str(waypoint[1]))

            angle_of_waypoints = self.get_angle_of_waypoints(self.actual_waypoint, waypoint)

            if (ranges["straight on"][0] <= angle_of_waypoints <= ranges["straight on"][1] or
                    ranges["straight on"][2] <= angle_of_waypoints <= ranges["straight on"][3]) and command == "straight on":
                filter_waypoints[command].append(waypoint)
                next_orientation = 0 + self.robot_z
                simple_rotate = 0
                commands[-1] = commands[-1]+'*'
            elif (ranges["go back"][0] <= angle_of_waypoints <= ranges["go back"][1] or
                  ranges["go back"][2] <= angle_of_waypoints <= ranges["go back"][3]) and command == "go back":
                filter_waypoints[command].append(waypoint)
                next_orientation = 180 + self.robot_z
                simple_rotate = 180
                commands[-1] = commands[-1]+'*'
            elif (ranges["right"][0] <= angle_of_waypoints <= ranges["right"][1] or
                  ranges["right"][2] <= angle_of_waypoints <= ranges["right"][3]) and command == "right":
                filter_waypoints[command].append(waypoint)
                next_orientation = -90 + self.robot_z
                simple_rotate = -90
                commands[-1] = commands[-1]+'*'
            elif (ranges["left"][0] <= angle_of_waypoints <= ranges["left"][1] or
                  ranges["left"][2] <= angle_of_waypoints <= ranges["left"][3]) and command == "left":
                filter_waypoints[command].append(waypoint)
                next_orientation = 90 + self.robot_z
                simple_rotate = 90
                commands[-1] = commands[-1]+'*'

        #self.plot_waypoints(points=self.waypoints,labels1=distances,labels2=commands)
        # Find the closest waypoint from the filtered waypoints
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
        '''
        Navigates the robot based on the given command and actual waypoint.

        Args:
            command: The command indicating the desired direction.
            actual_waypoint: The actual waypoint or the string "real" to obtain the robot's current position.

        Returns:
            The coordinates of the next waypoint.
            The next quaternion orientation.
            The rotation angle for simple rotation.
            The actual orientation before adjusting for multiple waypoints.
        '''
        self.actual_waypoint = actual_waypoint
        if self.actual_waypoint == "real":
            self.get_robot_position(True)  # Obtain the robot's current position
        self.next_waypoint, next_orientation, rotate_angle, actual_orientation = self.find_closest_waypoint(command)
        print(f"actual waypoint: {self.actual_waypoint}, next waypoint: {self.next_waypoint}")
        return self.next_waypoint, next_orientation, rotate_angle, actual_orientation
