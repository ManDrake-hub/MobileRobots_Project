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

class Move:
    def __init__(self) -> None:
        self.msg = PoseStamped()
        self.control_robot = RobotController("/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv")
        self.calibration_service = rospy.ServiceProxy('calibration_server', Calibration)
        rospy.wait_for_service('calibration_server')
        #self.sub = rospy.Subscriber("qr_most_common", String, self.callback_command)
        self.sub = rospy.Subscriber("qr_data_topic", String, self.callback_command)
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        self.waiting = False
        self.next_waypoint = None
        self.command = None
        self.thr = 0.8
        rospy.sleep(3.0)
 
    # Refactor command
    def callback_command(self, msg):
        #TO DO: fix empty data
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
        self.msg.pose.orientation.z = orientation[2]
        self.msg.pose.orientation.w = orientation[3]
        self.pub_goal.publish(self.msg)
        rospy.sleep(3.0)
        #print(f"{self.msg.__str__()}")

    # TO DO: call QR service to return command
    # Move the robot to the goal and return the command recognized
    def goal_reached(self,next_goal):
        self.send_goal(next_goal[0],next_goal[1],quaternion_from_euler(0,0,next_goal[2]))
        rospy.loginfo("Goal SEND")
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.loginfo("Goal REACHED")

    # Move the robot to the nearest waypoints due to the command. 
    # If the command is specified, use this. Otherwise take it from the topic. 
    # If you can't find any command from the topic, wait for them to reposition you and look for another one going straight
    def move(self, command = None):
        if command != None:
            self.command = command
        print(f"COMMAND: {self.command}")
        if self.command == None:
            rospy.loginfo("Startup the robot position then press ENTER")
            input()
            self.calibration()
            self.command = "straight on"
        if self.command != 'stop':
            self.next_waypoint = self.control_robot.navigate(self.command)
            self.command = None
            self.goal_reached(self.next_waypoint)
            #self.pub_next_waypoint.publish(Int32MultiArray(data=self.next_waypoint))
        else:
            print("FINISH")

if __name__ == "__main__":
    rospy.init_node("goal_custom")
    state = None
    navigation = Move()
    rate = rospy.Rate(10.0)
    navigation.calibration()
    navigation.move("straight on")
    while state != "FINISH":
        state = navigation.move()
    rospy.spin()