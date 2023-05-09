#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

###################### TO-DO as a Service
def move_robot(request):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    forward_msg = Twist()
    forward_msg.linear.x = 1 
    backward_msg = Twist()
    backward_msg.linear.x = -1

    pub.publish(forward_msg)
    rospy.sleep(2)  
    pub.publish(backward_msg)
    rospy.sleep(2)  

    stop_msg = Twist()
    pub.publish(stop_msg)

    # Ritorna una EmptyResponse per segnalare il completamento del servizio
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('calibration')
    s = rospy.Service('calibration', Empty, move_robot)
    rospy.loginfo("Move robot service ready.")
    rospy.spin()