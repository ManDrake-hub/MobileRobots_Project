#! /usr/bin/python3
import rospy
from std_msgs.msg import Empty

def clear_costmap():
    costmap_clear_pub = rospy.Publisher('/move_base/clear_costmaps', Empty, queue_size=1)
    empty_msg = Empty()
    print("pulisco")
    costmap_clear_pub.publish(empty_msg)

def periodic_cleanup(event):
    clear_costmap()

rospy.init_node('clear')
cleanup_timer = rospy.Timer(rospy.Duration(4), periodic_cleanup)
rospy.spin()

