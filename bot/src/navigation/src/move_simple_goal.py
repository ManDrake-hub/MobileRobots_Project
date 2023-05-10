#! /usr/bin/python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import csv
import os
from std_msgs.msg import String, Int32MultiArray
import random
from geometry_msgs.msg import Twist
import numpy as np 
import math
import tf
from navigation.srv import Calibration
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Move:
    def __init__(self) -> None:
        self.msg = PoseStamped()
        self.waiting = False
        self.next_command = None
        self.next_waypoint = None
        self.next_orientation = None
        self.thr = 0.8
        self.listener = tf.TransformListener()
        self.calibration_service = rospy.ServiceProxy('calibration_server', Calibration)
        rospy.wait_for_service('calibration_server')
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pub_next_waypoint = rospy.Publisher('next_waypoint', Int32MultiArray, queue_size=10)
        #self.sub = rospy.Subscriber("qr_most_common", String, self.callback_command)
        self.sub = rospy.Subscriber("qr_data_topic", String, self.callback_command)
        rospy.sleep(3.0)

    # Incrementation of dict commands
    def callback_command(self, msg):
        self.next_command = msg.data.lower().replace("\u200b","")

    # Send next waypoint to reach
    def send_goal(self,x,y,orientation):
        self.msg.header.frame_id = "map"
        self.msg.pose.position.x = float(x)
        self.msg.pose.position.y = float(y)
        self.msg.pose.orientation.z = orientation[2]
        self.msg.pose.orientation.w = orientation[3]
        self.pub_goal.publish(self.msg)
        rospy.sleep(3.0)
        #print(f"{self.msg.__str__()}")
    
    # Calibrate robot 
    def calibration(self):
        # Calibration
        answer = self.calibration_service().answer
        return answer

    def compute_magnitude_angle_with_sign(self,target_location, current_location, orientation):
        target_vector = np.array([target_location[0] - current_location[0], target_location[1] - current_location[1]])
        norm_target = np.linalg.norm(target_vector)
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        d_angle = math.degrees(math.atan2(np.cross(forward_vector, target_vector), np.dot(forward_vector, target_vector)))

        return (norm_target, d_angle)

    def find_closest_waypoint(self,command, current_location, orientation, waypoint_locations):
        waypoint_locations = [wp for wp in waypoint_locations for i in range(len(wp)) 
                              if self.next_waypoint is None or wp[i] != self.next_waypoint[i] or 
                              (current_location[i] < self.next_waypoint[i] - self.thr and wp[i] > self.next_waypoint[i] + self.thr)]
        # determina l'angolo minimo e massimo in base al comando
        if command == 'straight on':
            min_angle = -45
            max_angle = 45
        elif command == 'go back':
            min_angle = -135
            max_angle = 135
        elif command == 'right':
            min_angle = -135
            max_angle = -45
        elif command == 'left':
            min_angle = 45
            max_angle = 135
        else:
            raise ValueError("Comando non valido")

        # calcola l'angolo e la magnitudine per ogni waypoint
        magnitudes_angles = []
        for waypoint_location in waypoint_locations:
            magnitude, angle = self.compute_magnitude_angle_with_sign(waypoint_location, current_location, orientation)
            magnitudes_angles.append((magnitude, angle))

        # trova l'indice del waypoint più vicino in base all'angolo e alla magnitudine
        angles = np.array([ma[1] for ma in magnitudes_angles])
        mask = (angles >= min_angle) & (angles <= max_angle)
        filtered_magnitudes_angles = np.array(magnitudes_angles)[mask]
        if len(filtered_magnitudes_angles) == 0:
            print("WAYPOINT NOT FOUND")
            return None
        closest_index = np.argmin(filtered_magnitudes_angles[:, 0])
        closest_waypoint = np.array(waypoint_locations)[mask][closest_index]
        _, closest_angle = self.compute_magnitude_angle_with_sign(closest_waypoint, current_location, orientation)
        closest_quaternion = quaternion_from_euler(0, 0, math.radians(closest_angle))
        return closest_waypoint, closest_quaternion

    # Get robot position 
    def get_robot_position(self):
        try:
            # ------- Cerca il transform tra i frame di riferimento della posizione del robot e della mappa
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            rot = tf.transformations.euler_from_quaternion(rot)
            print('Robot position: {}'.format(trans))
            print('Robot orientation: {}'.format(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return trans, rot

    # TO DO: call QR service to return command
    # Move the robot to the goal and return the command recognized
    def goal_reached(self,next_goal,orientation):
        self.send_goal(next_goal[0],next_goal[1],orientation)
        rospy.loginfo("Goal send")
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.loginfo("Goal reached")
 
    def move(self,command):
        print(f"COMMAND: {command}")
        if command == None:
            rospy.loginfo("Startup the robot position then press enter")
            input()
            self.calibration()
            command = "straight on"
        trans, rot = self.get_robot_position()
        if command != 'stop':
            self.next_waypoint, self.next_orientation = self.find_closest_waypoint(command, trans, rot[2], waypoints)
            print(f"Next waypoint: {self.next_waypoint}")
            self.goal_reached(self.next_waypoint, self.next_orientation)
            self.pub_next_waypoint.publish(Int32MultiArray(data=self.next_waypoint))
        else:
            print("Finish")

if __name__ == "__main__":
    rospy.init_node("goal_custom")
    
    navigation = Move()
    rate = rospy.Rate(10.0)
    waypoints = []
    # Read waypoints
    with open('/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            waypoints.append((float(row[0]), float(row[1])))

    navigation.calibration()
    navigation.move('straight on')
    navigation.move('right')
    navigation.move('right')
    navigation.move(navigation.next_command)
    rospy.spin()