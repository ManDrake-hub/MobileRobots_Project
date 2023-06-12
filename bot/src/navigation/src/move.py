#!/usr/bin/python3

import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import os
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf
from navigation.srv import Calibration
from tf.transformations import euler_from_quaternion
from robot_controller import RobotController
from camera.srv import QR
from dynamic_reconfigure.client import Client
from robot_controller import RobotController

class Move:
    def __init__(self):
        # Initialize the Move class
        self.msg = PoseStamped()
        self.pose_estimate = PoseWithCovarianceStamped()

        # Create an instance of the RobotController class
        self.control_robot = RobotController(os.path.dirname(__file__) + "/waypoints.csv")
        self.real_robot = RobotController(os.path.dirname(__file__) + "/waypoints.csv")
        self.calibration_service = rospy.ServiceProxy('calibration_server', Calibration)
        rospy.wait_for_service('calibration_server')
        self.QR_service = rospy.ServiceProxy('QR_command', QR)
        rospy.wait_for_service('QR_command')

        # Subscribe to relevant topics and publish to others
        self.qr_sub = rospy.Subscriber('qr_data_topic', String, self.callback_qr)
        self.sub_stop = rospy.Subscriber("stop", Bool, self.callback_recovery)
        self.pub_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_rot = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_pose_estimate = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        # Set parameters for robot movement
        self.rot_speed = 0.5
        self.update_step = 0.01
        self.actual_waypoint = None
        self.next_waypoint = None
        self.command = None
        self.actual_goal = None
        self.fake_goal = False
        self.set_amcl_params()
        self.set_move_base_params()

    def callback_qr(self, msg):
        return
        # Callback function for QR data
        #self.set_fast()

    def callback_recovery(self, msg):
        # Callback function for recovery
        self.real_robot.get_robot_position(False)
        if msg.data:
            stop_msg = Twist()
            self.pub_rot.publish(stop_msg)
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.real_robot.robot_x
            goal.pose.position.y = self.real_robot.robot_y
            goal.pose.orientation.x = self.real_robot.robot_z[0]
            goal.pose.orientation.y = self.real_robot.robot_z[1]
            goal.pose.orientation.z = self.real_robot.robot_z[2]
            goal.pose.orientation.w = self.real_robot.robot_z[3]
            self.fake_goal = True
            self.pub_goal.publish(goal)
            return
        self.pose_estimate.header.frame_id = "map"
        self.pose_estimate.pose.pose.position.x = self.real_robot.robot_x - 0.5
        self.pose_estimate.pose.pose.position.y = self.real_robot.robot_y - 0.5
        self.pose_estimate.pose.pose.orientation.x = self.real_robot.robot_z[0]
        self.pose_estimate.pose.pose.orientation.y = self.real_robot.robot_z[1]
        self.pose_estimate.pose.pose.orientation.z = self.real_robot.robot_z[2]
        self.pose_estimate.pose.pose.orientation.w = self.real_robot.robot_z[3]
        self.pose_estimate.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.pub_pose_estimate.publish(self.pose_estimate)
        rospy.sleep(0.5)
        self.pub_goal.publish(self.actual_goal)
        rospy.sleep(0.5)
        self.fake_goal = False

    def calibration(self):
        # Calibrate the robot
        answer = self.calibration_service().answer
        return answer

    def set_amcl_params(self):
        # Set parameters for amcl node
        client = Client("amcl")
        params = {
            'min_particles': 1000,
            'force_update_after_initialpose': True,
            'force_update_after_set_map': True,
            'update_min_d': 0.1,
            'update_min_a': 0.1,
            'gui_publish_rate': 10.0
        }
        client.update_configuration(params)

    def set_move_base_params(self):
        # Set parameters for move_base node
        client = Client("move_base/DWAPlannerROS")
        params = {'xy_goal_tolerance': 0.5, 'yaw_goal_tolerance': 3.14}
        client.update_configuration(params)

        client = Client("move_base/global_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/global_costmap/inflation_layer")
        params = {'inflation_radius': 1.0}
        client.update_configuration(params)
        client = Client("move_base/local_costmap")
        params = {'transform_tolerance': 0.5}
        client.update_configuration(params)
        client = Client("move_base/local_costmap/inflation_layer")
        params = {'inflation_radius': 0.7}
        client.update_configuration(params)

    def set_slow(self):
        # Set slow movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
        params = {
            'max_vel_x': 0.15,
            'min_vel_x': -0.15,
            'max_vel_trans': 0.15,
            'min_vel_trans': 0.08,
            'max_vel_theta': 1.0,
            'min_vel_theta': 0.5,
            'acc_lim_x': 1.5,
            'acc_lim_theta': 2.5
        }
        client.update_configuration(params)

    def set_medium(self):
        # Set medium movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
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
        # Set fast movement parameters for the robot
        client = Client("move_base/DWAPlannerROS")
        params = {
            'max_vel_x': 0.26,
            'min_vel_x': -0.26,
            'max_vel_trans': 0.26,
            'min_vel_trans': 0.18,
            'max_vel_theta': 1.82,
            'min_vel_theta': 0.9,
            'acc_lim_x': 2.5,
            'acc_lim_theta': 3.2
        }
        client.update_configuration(params)

    def send_goal(self, x, y, orientation):
        # Send a goal position to the robot
        self.msg.header.frame_id = "map"
        self.msg.pose.position.x = float(x)
        self.msg.pose.position.y = float(y)
        self.msg.pose.orientation.x = orientation[0]
        self.msg.pose.orientation.y = orientation[1]
        self.msg.pose.orientation.z = orientation[2]
        self.msg.pose.orientation.w = orientation[3]
        self.pub_goal.publish(self.msg)
        self.actual_goal = self.msg
        rospy.sleep(3.0)

    def rotate(self, angle):
        # Rotate the robot by a specified angle
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = math.copysign(self.rot_speed, angle)
        print(f"rotate {math.degrees(angle)}")
        delta = 0
        while True:
            self.pub_rot.publish(move)
            rospy.sleep(self.update_step)
            delta = delta + self.rot_speed * self.update_step
            if abs(delta) >= abs(angle):
                break
    
    def rotate_difference(self,angle,actual_orientation):
        self.real_robot.get_robot_position(False)
        real = self.real_robot.normalize(math.degrees(euler_from_quaternion(self.real_robot.robot_z)[2]))
        final_rotate = self.real_robot.normalize((actual_orientation - real) + angle)
        return final_rotate
        
    def goal_reached(self, next_goal, orientation, angle, actual_orientation):
        _was_fake = False
        # Handle the situation when the goal is reached
        angle = self.rotate_difference(angle,actual_orientation)
        self.rotate(math.radians(angle))
        self.send_goal(next_goal[0], next_goal[1], orientation)
        rospy.loginfo("Goal SEND")
        rospy.wait_for_message("move_base/result", MoveBaseActionResult)

        while self.fake_goal:
            _was_fake = True
            rospy.sleep(0.5)
        
        if _was_fake:
            rospy.wait_for_message("move_base/result", MoveBaseActionResult)
        command = self.QR_service().answer
        self.command = command.data
        #navigation.set_medium()
        rospy.loginfo("Goal REACHED")

    def move(self, command=None, real=None):
        # Move the robot to the specified goal
        if command is not None:
            self.command = command
        if real is not None:
            self.actual_waypoint = real

        if self.command != 'stop' and self.command != "":
            self.next_waypoint, next_orientation, rotate_angle, actual_orientation = self.control_robot.navigate(self.command, self.actual_waypoint)
            self.command = None
            if self.next_waypoint is not None:
                self.goal_reached(self.next_waypoint, next_orientation, rotate_angle, actual_orientation)
                self.actual_waypoint = self.next_waypoint
            else:
                rospy.loginfo("I haven't found a waypoint")
                self.command = "stop"
        else:
            print("FINISH")
            return "FINISH"

if __name__ == "__main__":
    rospy.init_node("move")
    state = None
    navigation = Move()
    navigation.set_fast()
    navigation.move("straight on", "real")
    while state != "FINISH":
        state = navigation.move()
    rospy.spin()
