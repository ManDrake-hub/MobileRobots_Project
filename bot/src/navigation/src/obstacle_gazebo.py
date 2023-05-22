#! /usr/bin/python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def spawn_udf_model():
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        model_path = "/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/oggetto.urdf"
        model_name = "my_object"
        reference_frame = "map"
        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        spawn_model(model_name, open(model_path).read(),'/object',initial_pose,reference_frame)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("spawn_udf_model_node")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    ##########OBJECT SPAWN#################
    spawn_udf_model()
    rospy.sleep(4)
    while rospy.is_shutdown:
        ##########OBJECT POSITIONING#################
        state = ModelState()
        state.model_name = 'my_object'
        state.pose = Pose(position=Point(5,0,0))
        set_model_state(state)
        rospy.loginfo('OBJECT MOVED.....')
    rospy.spin()

