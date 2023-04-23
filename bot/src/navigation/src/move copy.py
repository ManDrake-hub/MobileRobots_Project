#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from WaypointMovement import WaypointMovement
import numpy as np

# TO-DO: tests for exercises
if __name__ == "__main__":
    rospy.init_node("vel_publisher")

    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.sleep(3)

    wm = WaypointMovement(pub, noise=True, odom=False, kf=True)

    wm.set_waypoints([
        np.array((0.0, 0.0, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        np.array((2.0, 0.0, 0.0)),
        np.array((3.0, 0.0, 0.0)),
        np.array((4.0, 0.0, 0.0))
    ])
    wm.play(wait_user=False)