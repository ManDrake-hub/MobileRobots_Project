#! /usr/bin/python3
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import csv
import os
from std_msgs.msg import String, Int32MultiArray, Bool
import random
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np 
import math
import tf
from navigation.srv import Calibration
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from robot_controller import RobotController
from camera.srv import QR 
from dynamic_reconfigure.client import Client

class Move:
    def __init__(self) -> None:
        self.msg = PoseStamped()
        self.control_robot = RobotController(os.path.dirname(__file__) + "/waypoints.csv")
        self.calibration_service = rospy.ServiceProxy('calibration_server', Calibration)
        self.QR_service = rospy.ServiceProxy('QR_command',QR)
        self.qr_sub = rospy.Subscriber('qr_data_topic', String, self.callback_qr)
        rospy.wait_for_service('calibration_server')
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_pose_estimate = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pub_rot = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.sub_stop = rospy.Subscriber("stop", Bool, self.callback_recovery)
        self.listener = tf.TransformListener()
        self.pose_estimate = PoseWithCovarianceStamped()
        self.rot_speed = 0.5
        self.update_step = 0.01
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.actual_goal = None
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.goal_orientation = None
        self.last_stop = False
        rospy.sleep(3.0)

    def callback_qr(self,msg):
        self.set_fast()
    
    def callback_recovery(self,msg):
        #print("pronto per la recovery")

        if msg:
            stop_msg = Twist()
            self.pub_rot.publish(stop_msg)
            return

        self.get_robot_position()
        self.pose_estimate.header
        self.pose_estimate.header.frame_id = "map"
        if self.robot_x or self.robot_y is not None:
            self.pose_estimate.pose.pose.position.x = self.robot_x - 0.5
            self.pose_estimate.pose.pose.position.y = self.robot_y - 0.5
            self.pose_estimate.pose.pose.orientation.x = self.robot_z[0]
            self.pose_estimate.pose.pose.orientation.y = self.robot_z[1]
            self.pose_estimate.pose.pose.orientation.z = self.robot_z[2]
            self.pose_estimate.pose.pose.orientation.w = self.robot_z[3]
            self.pose_estimate.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            print("pubblico nuova posa")
            self.pub_pose_estimate.publish(self.pose_estimate)
            rospy.sleep(0.5)
        self.pub_goal(self.actual_goal)
        
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
        print(f"orientation final goal {math.degrees(euler_from_quaternion(orientation)[2])}")
        self.msg.pose.orientation.x = orientation[0]
        self.msg.pose.orientation.y = orientation[1]
        self.msg.pose.orientation.z = orientation[2]
        self.msg.pose.orientation.w = orientation[3]
        self.pub_goal.publish(self.msg)
        self.actual_goal = self.msg
        rospy.sleep(3.0)
        #print(f"{self.msg.__str__()}")
    
    def set_amcl_params(self):
        client = Client("amcl")
        params = {'min_particles': 1000}
        client.update_configuration(params)
        params = {'force_update_after_initialpose': True}
        client.update_configuration(params)
        params = {'force_update_after_set_map': True}
        client.update_configuration(params)
        params = {'update_min_d': 0.1}
        client.update_configuration(params)
        params = {'update_min_a': 0.1}
        client.update_configuration(params)
        params = {'gui_publish_rate': 10.0}
        client.update_configuration(params)
    
    def set_move_base_params(self):
        client = Client("move_base/DWAPlannerROS")
        params = {'xy_goal_tolerance': 0.5}
        client.update_configuration(params)
        params = {'yaw_goal_tolerance': 3.14}
        client.update_configuration(params)
        
        client = Client("move_base/global_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/global_costmap/inflation_layer")
        params = {'inflation_radius': 0.5}
        client.update_configuration(params)
        client = Client("move_base/local_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/local_costmap/inflation_layer")
        params = {'inflation_radius': 1.0}
        client.update_configuration(params)
        
    def set_slow(self):
        client = Client("move_base/DWAPlannerROS")
        params = {'max_vel_x': 0.15, 'min_vel_x': -0.15, 'max_vel_trans':0.15, 'min_vel_trans': 0.08, 'max_vel_theta':1.0, 'min_vel_theta': 0.5, 'acc_lim_x':1.5, 'acc_lim_theta': 2.5}
        client.update_configuration(params)

    def set_fast_sl(self):
        print("slow")
        client = Client("move_base/DWAPlannerROS")
        #params = {'max_vel_x': 0.26, 'min_vel_x': -0.26, 'max_vel_trans':0.26, 'min_vel_trans': 0.18, 'max_vel_theta':1.82, 'min_vel_theta': 0.9, 'acc_lim_x':2.5, 'acc_lim_theta': 3.2}
        params = {
            'max_vel_x': 0.2,
            'min_vel_x': -0.2,
            'max_vel_trans': 0.2,
            'min_vel_trans': 0.14,
            'max_vel_theta': 1.5,
            'min_vel_theta': 0.75,
            'acc_lim_x': 2.0,
            'acc_lim_theta': 2.5
        }   
        client.update_configuration(params)

    def set_fast(self):
        client = Client("move_base/DWAPlannerROS")
        params = {'max_vel_x': 0.26, 
                  'min_vel_x': -0.26, 
                  'max_vel_trans':0.26, 
                  'min_vel_trans': 0.18, 
                  'max_vel_theta':1.82, 
                  'min_vel_theta': 0.9, 
                  'acc_lim_x':2.5, 
                  'acc_lim_theta': 3.2}
        client.update_configuration(params)

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
            self.robot_z = math.degrees(rot[2])
            self.actual_waypoint = (self.robot_x,self.robot_y)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    
    def normalize(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def rotate(self,angle):
        self.get_robot_position()
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        if self.goal_orientation is None:
            goal = 0
        else:
            goal = math.degrees(euler_from_quaternion(self.goal_orientation)[2])
        real_angle = self.robot_z
        print(f"goal orientation: {goal}, real angle {real_angle}")
        new_angle = math.degrees(angle)+(goal-real_angle)
        move.angular.z = math.copysign(self.rot_speed,math.radians(self.normalize(new_angle)))
        print(f"initial angle {math.degrees(angle)}")
        print(f"rotate {new_angle}")
        delta = 0
        while True:
            self.pub_rot.publish(move)
            rospy.sleep(self.update_step)
            delta = delta + self.rot_speed * self.update_step
            if abs(delta) >= abs(math.radians(self.normalize(new_angle))):
                break
        
    # Move the robot to the goal and return the command recognized
    def goal_reached(self,next_goal, next_command,orientation,angle):
        self.rotate(math.radians(angle))
        self.send_goal(next_goal[0],next_goal[1],orientation)
        self.goal_orientation = orientation
        rospy.loginfo("Goal SEND")
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        command = self.QR_service().answer 
        self.command = command.data 
        navigation.set_fast_sl()
        #rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        rospy.loginfo("Goal REACHED")

    # Move the robot to the nearest waypoints due to the command. 
    # If the command is specified, use this. Otherwise take it from the topic. 
    # If you can't find any command from the topic, wait for them to reposition you and look for another one going straight
    def move(self, command = None, real = None):
        if command != None:
            self.command = command
        if real != None:
            self.actual_waypoint = real
        print(f"COMMAND: {self.command}")
        # TODO: DO OR NOT
        '''
        if self.command == "":
            rospy.loginfo("Startup the robot position then press ENTER")
            input()
            #self.calibration()
            self.pub_goal.publish(self.actual_goal)
        '''
        if self.command != 'stop' or self.command != "":
            self.next_waypoint, next_command,orientation,angle= self.control_robot.navigate(self.command, self.actual_waypoint)
            self.command = None
            if self.next_waypoint is not None:
                self.goal_reached(self.next_waypoint, next_command,orientation,angle)
                self.actual_waypoint = self.next_waypoint
            else:
                rospy.loginfo("I haven't find a waypoint")
                self.command = "stop"
            #self.pub_next_waypoint.publish(Int32MultiArray(data=self.next_waypoint))
        else:
            print("FINISH")
            return "FINISH"
        

if __name__ == "__main__":
    rospy.init_node("goal_custom")
    rospy.wait_for_service('QR_command') 
    state = None
    navigation = Move()
    rate = rospy.Rate(10.0)
    navigation.set_amcl_params()
    navigation.set_move_base_params()
    #navigation.calibration()
    #print("END CALIBRATION")
    navigation.set_fast_sl()
    navigation.move("straight on","real")
    while state != "FINISH":
        state = navigation.move()
    rospy.spin()
