#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from WaypointMovement import WaypointMovement
import numpy as np

if __name__ == "__main__":
    rospy.init_node("exercise_3")

    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.sleep(3)

    wm = WaypointMovement(pub, noise=False, odom=True, kf=False, ekf=False, mc=(False,1))

    wm.set_waypoints([
        np.array((0.0, 0.0, 0.0)),
        np.array((0.5, 0.0, 0.0)),
        np.array((0.5, -0.5, 0.0)),
        np.array((1.0, -0.5, 0.0)),
        np.array((1.0, 0.0, 0.0)),
        np.array((1.0, 0.5, 0.0))
    ])
    wm.play(wait_user=False)