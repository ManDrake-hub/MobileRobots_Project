#! /usr/bin/python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from KalmanFilter import KalmanFilter
from nav_msgs.msg import Odometry
import random
from typing import List, Tuple
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class WaypointMovement:
    def __init__(self, publisher: rospy.Publisher, pose_start: np.ndarray=np.array((0.0, 0.0, 0.0)), noise=False, noise_var=0.25, odom=False, kf=False) -> None:
        #####################################
        # Don't touch this section
        self.publisher = publisher
        self._waypoints: List = None
        self._waypoint_current_index = 0
        self._belief: np.ndarray = pose_start
        self._odom: np.ndarray = pose_start.copy()
        self._kf: KalmanFilter = KalmanFilter()
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        #####################################
        # Performance params of our robot
        self.speed_linear_max = 0.26
        self.speed_rad_max = 0.1

        self.stop_time=0.5
        self.time_to_reach_max_speed = 2
        self.time_to_stop = 1
        self.update_step = 0.01
        #####################################
        # Noise params
        self.noise = noise
        self.noise_var = noise_var
        #####################################
        # Corrections
        self.odom = odom
        self.kf = kf
        #####################################
        self.rate = rospy.Rate(1 / self.update_step)

    #########################################
    # Setters
    def set_waypoints(self, waypoints: List[np.ndarray], waypoint_current_index: int=0) -> None:
        self._waypoints = waypoints
        self._waypoint_current_index = waypoint_current_index

    def set_belief(self, belief: np.ndarray) -> None:
        self._belief = belief

    #########################################
    # Getters
    def get_noise(self):
        return np.random.normal(size=(3, ), scale=self.noise_var)

    def get_belief(self) -> np.ndarray:
        return self._belief

    #########################################
    # Utils
    def calculate_time(self, delta: float, speed: float):
        t = delta / speed
        return t

    def rad_to_degree(self, rad) -> float:
        return math.degrees(rad)

    def degree_to_rad(self, degree) -> float:
        return math.radians(degree)

    def rad_for_point(self, point: np.ndarray) -> float:
        start_x, start_y, _ = self._belief
        end_x, end_y = point
        rad = math.atan2((end_y - start_y), (end_x - start_x))
        return rad

    def delta_to_stop(self, speed_current, speed_max):
        # Set starting conditions
        delta_to_stop = 0
        speed_current = abs(speed_current)
        # Until we stop
        while speed_current > 0:
            # Increase the space needed to stop by the amount of movement done
            delta_to_stop += speed_current * self.update_step
            # Decrease current speed by the max amount set
            speed_current -= speed_max / (self.time_to_stop / self.update_step)
        # Return the accumulated space
        return delta_to_stop

    def get_rotation(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion (orientation_list)
        return yaw

    def callback_odom(self, odom):
        self._odom[0] = odom.pose.pose.position.x
        self._odom[1] = odom.pose.pose.position.y
        self._odom[2] = self.get_rotation(odom.pose.pose.orientation)

    def get_odom_position(self) -> np.ndarray:
        return self._odom

    #########################################
    # Movement
    def stop(self):
        self._rotate(0)
        rospy.sleep(self.stop_time)

    def _rotate(self, speed):
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = speed
        self.publisher.publish(move)

    def _move_forward(self, speed):
        move = Twist()
        move.linear.x = speed
        move.linear.y = 0.0
        move.angular.z = 0.0
        self.publisher.publish(move)

    def move(self, delta_x, delta_z):
        assert delta_x==0 or delta_z == 0, "You can either move along x or rotate around z, not both"
        # Set values to match whether we want linear or rotation movement
        delta = delta_x if delta_x != 0 else delta_z
        speed_max = self.speed_linear_max if delta_x != 0 else self.speed_rad_max
        func = self._move_forward if delta_x != 0 else self._rotate

        # Starting speed and movement already done set to 0
        speed = 0
        _delta = 0
        
        while True:
            # Move or rotate with the current speed using the sign of the requested movement
            func(math.copysign(speed, delta))
            self.rate.sleep()

            # Update the movement already done
            _delta = _delta + speed * self.update_step
            # If we have reached our objective, stop the loop
            if abs(_delta) >= abs(delta):
                break
            
            # If the space left is higher than the one needed to stop from the current speed 
            if (delta - _delta) > self.delta_to_stop(speed, speed_max):
                # Increase speed until we reach max speed
                if speed < speed_max:
                    speed += speed_max / (self.time_to_reach_max_speed / self.update_step)
            # Else (i.e. the space left is less or equal the one needed to fully stop from our current speed) decrease our speed
            else:
                speed -= speed_max / (self.time_to_reach_max_speed / self.update_step)
        # Stop after every movement (probably not needed since we are stopping gently)
        self.stop()

    def move_to_next(self):
        assert self._waypoint_current_index < len(self._waypoints)
        waypoint_next = self._waypoints[self._waypoint_current_index+1]

        # Add noise if requested
        if self.noise:
            waypoint_next = waypoint_next + self.get_noise()
            print(waypoint_next)

        # Rotate towards point
        rad_expected_for_point = self.rad_for_point(waypoint_next[:2])
        rad_delta = rad_expected_for_point - self._belief[-1]
        print("calc", rad_expected_for_point, self._belief[-1], rad_delta)
        self.move(0, rad_delta)

        # Move towards point
        point_expected = waypoint_next[:2]
        point_delta = point_expected - self._belief[:2]
        point_delta = math.sqrt((point_delta[0]**2 + point_delta[1]**2))
        self.move(point_delta, 0)
        
        # Rotate to match pose requested
        rad_expected = waypoint_next[-1]
        rad_delta = rad_expected - rad_expected_for_point
        self.move(0, rad_delta)

        # Set belief as the ideal waypoint
        self.set_belief(self._waypoints[self._waypoint_current_index+1])
        
        # Increase index
        self._waypoint_current_index += 1

    #########################################
    # Main functions
    def play(self, wait_user: bool=False):
        for _ in range(len(self._waypoints) - 1):
            self.move_to_next()
            print("belief for next step", self.get_belief())
            if self.odom:
                self.set_belief(self.get_odom_position())
                print("odom for next step", self.get_belief())
            if self.kf:
                self.set_belief((self._kf.get_kf_position(self.get_belief(), self.speed_linear_max)))
                print("kf for next step", self.get_belief())
            if wait_user:
                input("Press any key to move to the next waypoint")