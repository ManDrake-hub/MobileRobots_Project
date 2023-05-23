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
from robot_controller import RobotController
from camera.srv import QR # aggiunta mia

class Move:
    def __init__(self) -> None:
        self.msg = PoseStamped()
        self.control_robot = RobotController("/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv")
        self.calibration_service = rospy.ServiceProxy('calibration_server', Calibration)
        self.QR_service = rospy.ServiceProxy('QR_command',QR)
        rospy.wait_for_service('calibration_server')
        #self.sub = rospy.Subscriber("qr_most_common", String, self.callback_command)
        #self.sub = rospy.Subscriber("qr_data_topic", String, self.callback_command)
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_rot = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rot_speed = 0.5
        self.update_step = 0.01
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.thr = 0.8
        rospy.sleep(3.0)
 
    # Refactor command
    def callback_command(self, msg):
        #TO DO: fix empty data (if use OPENCV)
        #print(msg)
        self.command = msg.data.lower().replace("\u200b","")

    # Calibrate robot 
    def calibration(self):
        # Calibration
        answer = self.calibration_service().answer
        return answer
    
    # Send next waypoint to reach
    def send_goal(self,x,y,orientation):
        self.msg.header.frame_id = "map"
        self.msg.pose.position.x = float(x)
        self.msg.pose.position.y = float(y)
        print(f"goal finale {euler_from_quaternion(orientation)}")
        self.msg.pose.orientation.x = orientation[0]
        self.msg.pose.orientation.y = orientation[1]
        self.msg.pose.orientation.z = orientation[2]
        self.msg.pose.orientation.w = orientation[3]
        self.pub_goal.publish(self.msg)
        rospy.sleep(3.0)
        #print(f"{self.msg.__str__()}")
    
    def rotate(self,angle):
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = math.copysign(self.rot_speed,angle)
        print(f"ruoto di {math.degrees(angle)}")
        delta = 0
        while True:
            self.pub_rot.publish(move)
            rospy.sleep(self.update_step)
            delta = delta + self.rot_speed * self.update_step
            #rospy.loginfo(f'angle = {angle} delta = {delta}')
            if abs(delta) >= abs(angle):
                break
        

    # TO DO: call QR service to return command
    # Move the robot to the goal and return the command recognized
    def goal_reached(self,next_goal, next_command,orientation,angle):
        """if next_command == "straight_on":
            self.send_goal(next_goal[0],next_goal[1],quaternion_from_euler(0,0,0))
            self.control_robot.orientation = 0
        if next_command ==callback_qrobot.orientation = 90
        if next_command == "right":
            self.send_goal(next_goal[0],next_goal[1],quaternion_from_euler(0,0,-90)) # -90
            self.control_robot.orientation = -90
        if next_command == "go_back":
            self.send_goal(next_goal[0],next_goal[1],quaternion_from_euler(0,0,180)) # 180
            self.control_robot.orientation = 180
        rospy.loginfo("Goal SEND")
        command = self.QR_service().answer # aggiunta mia
        self.command = command.data # aggiunta mia 
        #rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.loginfo("Goal REACHED")"""

        if next_command == "straight_on":
            self.rotate(math.radians(angle))
            self.send_goal(next_goal[0],next_goal[1],orientation)
        if next_command == "left":
            self.rotate(math.radians(angle))
            print(f"orientation del send goal {orientation}")
            self.send_goal(next_goal[0],next_goal[1],orientation) # 90
        if next_command == "right":
            self.rotate(math.radians(angle))
            self.send_goal(next_goal[0],next_goal[1],orientation) # -90
        if next_command == "go_back":
            self.rotate(math.radians(angle))
            self.send_goal(next_goal[0],next_goal[1],orientation) # 180
        rospy.loginfo("Goal SEND")
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        command = self.QR_service().answer # aggiunta mia
        self.command = command.data #calibration aggiunta mia 
        #rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.loginfo("Goal REACHED")


     # TO DO: manage not la0.0, -0.0, 0.0st qr code
    # Move the robot to the nearest waypoints due to the command. 
    # If the command is specified, use this. Otherwise take it from the topic. 
    # If you can't find any command from the topic, wait for them to reposition you and look for another one going straight
    def move(self, command = None, real = None):
        if command != None:
            self.command = command
        if real != None:
            self.actual_waypoint = real
        print(f"COMMAND: {self.command}")
        if self.command == None:
            rospy.loginfo("Startup the robot position then press ENTER")
            input()
            self.calibration()
            self.actual_waypoint = real
            self.command = "straight_on"
        if self.command != 'stop':
            self.next_waypoint, next_command,orientation,angle = self.control_robot.navigate(self.command, self.actual_waypoint)
            self.command = None # ?
            if self.next_waypoint is not None:
                self.goal_reached(self.next_waypoint, next_command,orientation,angle)
                self.actual_waypoint = self.next_waypoint
            #self.pub_next_waypoint.publish(Int32MultiArray(data=self.next_waypoint))
        else:
            print("FINISH")

if __name__ == "__main__":
    rospy.init_node("goal_custom")
    rospy.wait_for_service('QR_command') # aggiunta mia 
    print('QR command service started.......\n') # aggiunta mia
    state = None
    navigation = Move()
    rate = rospy.Rate(10.0)
    navigation.calibration()

    print("END CALIBRATION")
    
    navigation.move("straight_on","real")
    navigation.move(navigation.command)
    #navigation.move("right")
    #navigation.move("right")
    #navigation.move("right")
    #navigation.move("straight_on")
    while state != "FINISH":
        state = navigation.move()
    rospy.spin()
