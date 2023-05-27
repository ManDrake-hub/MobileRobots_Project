#! /usr/bin/python3
import rospy
import requests
from geometry_msgs.msg import Twist
from navigation.srv import Calibration, CalibrationResponse
import math
from robot_controller import RobotController
from geometry_msgs.msg import PoseArray

def calculate_particle_density(msg):
    num_poses = len(msg.poses)
    density = num_poses / msg.header.stamp.to_sec()  # Calcolo della densità
    #rospy.loginfo("Density: %.2f p/s", density)


def handle_service(req):
    waypoint_movement = WaypointMovement()
    waypoint_movement.move(0.2,0)
    waypoint_movement.move(-0.2,0)
    waypoint_movement.move(0.2,0)
    waypoint_movement.move(-0.2,0)
    waypoint_movement.move(0,math.radians(360))
    #waypoint_movement.move(0,-0.5)
    response = CalibrationResponse()
    response.answer.data = "Done"
    return response

class WaypointMovement:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=25)
        rospy.Subscriber('particlecloud', PoseArray, calculate_particle_density)
        self.speed_linear_max = 0.26
        self.speed_rad_max = 0.35
        self.stop_time=1
        self.time_to_reach_max_speed = 1
        self.time_to_stop = 0.5
        self.update_step = 0.01

    def stop(self):
        self._rotate(0)
        rospy.sleep(self.stop_time)

    def _rotate(self, speed):
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = speed
        self.pub.publish(move)

    def _move_forward(self,speed):
            move = Twist()
            move.linear.x = speed
            move.linear.y = 0.0
            move.angular.z = 0.0
            self.pub.publish(move)

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
    
    def move(self,delta_x, delta_z):
        assert delta_x==0 or delta_z == 0, "You can either move along x or rotate around z, not both"
        # Set values to match whether we want linear or rotation movement
        delta = delta_x if delta_x != 0 else delta_z
        speed_max = self.speed_linear_max if delta_x != 0 else self.speed_rad_max
        func = self._move_forward if delta_x != 0 else self._rotate

        # Starting speed and movement already done set to 0
        speed = speed_max
        _delta = 0
        
        while True:
            # Move or rotate with the current speed using the sign of the requested movement
            func(math.copysign(speed, delta))
            # self.rate.sleep()
            rospy.sleep(self.update_step)

            # Update the movement already done
            _delta = _delta + speed * self.update_step
            # If we have reached our objective, stop the loop
            if abs(_delta) >= abs(delta):
                break
        # Stop after every movement
        self.stop()

def main():
    rospy.init_node('calibration')
    s = rospy.Service('calibration_server', Calibration, handle_service)
    rospy.loginfo("Move robot service ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        print("Calibration server init has failed: %s"%e)
   