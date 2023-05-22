#! /usr/bin/python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def spawn_udf_model(name,position):
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        model_path = "/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/oggetto.urdf"
        model_name = name
        reference_frame = "map"
        initial_pose = Pose()
        initial_pose.position.x = position[0]
        initial_pose.position.y = position[1]
        initial_pose.position.z = position[2]
        spawn_model(model_name, open(model_path).read(),'/object',initial_pose,reference_frame)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("spawn_udf_model_node")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    points_list = [(5,0,0),(15.7,-5,0),(15.2,-2.5,0),(8.25,-9.5,0),(2.2,-10.9,0),(-4.75,-8.35,0),(24.5,-9.25,0),(36.5,-8.4,0),(32.4,-1.05,0),(21.85,-1.4,0)]


    #########OBSTACLES SPAWNING######################
    for i in range(len(points_list)):
        spawn_udf_model(name='object'+str(i),position=points_list[i])

    ##########OBJECT SPAWN#################
    #spawn_udf_model()
    #rospy.sleep(4)
    """while rospy.is_shutdown:
        ##########OBJECT POSITIONING#################
        state = ModelState()
        state.model_name = 'my_object'
        state.pose = Pose(position=Point(5,0,0))
        set_model_state(state)
        rospy.loginfo('OBJECT MOVED.....')
    rospy.spin()"""

