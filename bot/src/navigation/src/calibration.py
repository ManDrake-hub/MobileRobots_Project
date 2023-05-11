#! /usr/bin/python3
import rospy
import requests
from geometry_msgs.msg import Twist
from navigation.srv import Calibration, CalibrationResponse

def handle_service(req):
    response = CalibrationResponse()
    forward_msg = Twist()
    backward_msg = Twist()
    rotation_right = Twist()
    rotation_left = Twist()
    forward_msg.linear.x = 0.2
    backward_msg.linear.x = -0.2
    pub.publish(forward_msg)
    rospy.sleep(3)  
    pub.publish(backward_msg)
    rospy.sleep(3)
    forward_msg.linear.x = 0.3
    backward_msg.linear.x = -0.3
    pub.publish(forward_msg)
    rospy.sleep(3)  
    pub.publish(backward_msg)
    rospy.sleep(3)
    
    rotation_right.angular.z = 0.5
    rotation_left.angular.z = -0.5
    pub.publish(rotation_right)
    rospy.sleep(3)  
    pub.publish(rotation_left)
    rospy.sleep(3) 

    stop_msg = Twist()
    pub.publish(stop_msg)
    response.answer.data = "Done"
    return response

def main():
    rospy.init_node('calibration')
    s = rospy.Service('calibration_server', Calibration, handle_service)
    rospy.loginfo("Move robot service ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        main()
    except rospy.ROSInterruptException as e:
        print("Calibration server init has failed: %s"%e)
   