#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from WaypointMovement import WaypointMovement
import numpy as np
from gazebo_msgs.srv import GetModelState, SetModelState
import os
import pathlib
import glob


if __name__ == "__main__":
    #####################################################
    # MonteCarlo Settings
    FOLDER = "/MonteCarlo/exercise_2/"
    RUNS = 1

    #####################################################
    # Waffle Settings
    NOISE = True
    ODOM = False
    ODOM_TOPIC = "odom" # or "odom_noised"
    KF = False
    EKF = False
    WAIT_USER = False

    #####################################################
    # Rospy
    rospy.init_node("exercise_2")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=20)
    rospy.sleep(3)

    rospy.wait_for_service('/gazebo/get_model_state')
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.wait_for_service('/gazebo/set_model_state')
    sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    #####################################################
    # MonteCarlo setup
    os.makedirs(str(pathlib.Path(__file__).parent.resolve()) + FOLDER, exist_ok=True)
    files = glob.glob(str(pathlib.Path(__file__).parent.resolve()) + f"{FOLDER}/*")
    for file in files:
        os.remove(file)

    #####################################################
    # Run
    for i in range(RUNS):
        wm = WaypointMovement(pub, noise=NOISE, odom=ODOM, kf=KF, ekf=EKF, odom_topic=ODOM_TOPIC, 
                              get_state_proxy=gms, set_state_proxy=sms,
                              save_position_file=str(pathlib.Path(__file__).parent.resolve()) + f"{FOLDER}{i}.json")
        wm.set_gazebo_position(np.array([0.0, 0.0, 0.0]))

        wm.set_waypoints([
            np.array((0.0, 0.0, 0.0)),
            np.array((0.5, 0.0, 0.0)),
            np.array((0.5, -0.5, 0.0)),
            np.array((1.0, -0.5, 0.0)),
            np.array((1.0, 0.0, 0.0)),
            np.array((1.0, 0.5, 0.0))
        ])
        wm.play(wait_user=WAIT_USER)