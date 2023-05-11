#! /usr/bin/python3
import rospy
import requests
from geometry_msgs.msg import Twist
from navigation.srv import Calibration, CalibrationResponse
import math
from robot_controller import RobotController

def handle_service(req):
    waypoint_movement = WaypointMovement()
    waypoint_movement.move(0.2,0)
    waypoint_movement.move(-0.2,0)
    waypoint_movement.move(0.2,0)
    waypoint_movement.move(-0.2,0)
    waypoint_movement.move(0,0.5)
    waypoint_movement.move(0,-0.5)
    response = CalibrationResponse()
    response.answer.data = "Done"
    return response
'''
def handle_service_old(req):
    response = CalibrationResponse()
    forward_msg = Twist()
    backward_msg = Twist()
    rotation_right = Twist()
    rotation_left = Twist()
    c = 0
    forward_msg.linear.x = 0.2
    while c != 5:
        pub.publish(forward_msg) 
        rospy.sleep(0.1)  
        c +=1
    c = 0
    backward_msg.linear.x = -0.2
    while c != 5:
        pub.publish(backward_msg) 
        rospy.sleep(0.1)  
        c +=1
    c = 0
    rotation_right.angular.z = 0.5
    while c != 5:
        pub.publish(rotation_right) 
        rospy.sleep(0.1)  
        c +=1
    c = 0
    rotation_right.angular.z = -0.5
    while c != 5:
        pub.publish(rotation_right)
        rospy.sleep(0.1)  
        c +=1
    stop_msg = Twist()
    pub.publish(stop_msg)
    response.answer.data = "Done"
    return response
'''
class WaypointMovement:
    def __init__(self) -> None:
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.speed_linear_max = 0.26
        self.speed_rad_max = 0.35
        self.stop_time=1.0
        self.time_to_reach_max_speed = 3
        self.time_to_stop = 3.0
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
        speed = 0
        speed_prev = 0
        _delta = 0
        
        while True:
            # Move or rotate with the current speed using the sign of the requested movement
            func(math.copysign(speed, delta))
            if func == self._move_forward:
                #self._kf.update_position((speed - speed_prev)/self.update_step)
                speed_prev = speed
            # self.rate.sleep()
            rospy.sleep(self.update_step)

            # Update the movement already done
            _delta = _delta + speed * self.update_step
            # If we have reached our objective, stop the loop
            if abs(_delta) >= abs(delta):
                break
            
            # If the space left is higher than the one needed to stop from the current speed 
            if abs(delta - _delta) > abs(self.delta_to_stop(speed, speed_max)):
                # Increase speed until we reach max speed
                if speed < speed_max and delta > 0:
                    speed += speed_max / (self.time_to_reach_max_speed / self.update_step)
                elif speed < speed_max and delta < 0:
                    speed -= speed_max / (self.time_to_reach_max_speed / self.update_step)
            # Else (i.e. the space left is less or equal the one needed to fully stop from our current speed) decrease our speed
            else:
                if speed > 0 and delta > 0:
                    speed -= speed_max / (self.time_to_stop / self.update_step)
                elif speed < 0 and delta < 0:
                    speed += speed_max / (self.time_to_stop / self.update_step)
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
   