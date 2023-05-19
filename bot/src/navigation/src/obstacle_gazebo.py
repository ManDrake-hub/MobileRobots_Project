#! /usr/bin/python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_udf_model():
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        model_path = "/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/models/map/model_custom.sdf"
        model_name = "my_object"
        reference_frame = "map"
        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        spawn_model(model_name, model_path, reference_frame, initial_pose)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("spawn_udf_model_node")
    spawn_udf_model()
