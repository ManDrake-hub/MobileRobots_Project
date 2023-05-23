#! /usr/bin/python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


TIME_CHANGE = 3
FLAG = 0.1

def spawn_udf_model(name,position):
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        model_path = "/home/luigi/Scrivania/MobileRobots_Project/bot/src/navigation/src/oggetto.urdf"
        model_name = name
        reference_frame = "map"
        initial_pose = Pose()
        initial_pose.position.x = position[0]
        initial_pose.position.y = position[1]
        initial_pose.position.z = position[2]
        spawn_model(model_name, open(model_path).read(),'/object',initial_pose,reference_frame)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def change_state(name,new_state):
    state = ModelState()
    state.model_name = name
    state.pose = Pose(position=Point(new_state[0],new_state[1],new_state[2]))
    set_model_state(state)

if __name__ == "__main__":
    rospy.init_node("spawn_udf_model_node")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    stop = 0

    points_spawn_list = [(5,0,0),(15.7,-5,0),(15.2,-2.5,0),(8.25,-9.5,0),(2.2,-10.9,0),(-4.75,-8.35,0),(24.5,-9.25,0),(36.5,-8.4,0),(32.4,-1.05,0),(21.85,-1.4,0)]
    offesets = [(3,0.15,0),(0,3,0),(0,-6,0),(-4,0,0),(-1,1,0),(0,4,0),(-4,-1,0),(-1,4,0),(-4,0,0),(-1,1,0)]
    points_list = [[5,0,0],[15.7,-5,0],[15.2,-2.5,0],[8.25,-9.5,0],[2.2,-10.9,0],[-4.75,-8.35,0],[24.5,-9.25,0],[36.5,-8.4,0],[32.4,-1.05,0],[21.85,-1.4,0]]
    #########OBSTACLES SPAWNING######################
    for i in range(len(points_list)):
        spawn_udf_model(name='object'+str(i),position=points_spawn_list[i])

    rospy.sleep(TIME_CHANGE)
    #########POSITION CHANGING########################
    while not rospy.is_shutdown():
        
        while stop != 10:
            for i in range(len(offesets)):
                points_list[i][0] = points_list[i][0]+FLAG*offesets[i][0]
                points_list[i][1] = points_list[i][1]+FLAG*offesets[i][1]
                points_list[i][2] = points_list[i][2]+FLAG*offesets[i][2]
                change_state(name='object'+str(i),new_state=(points_list[i][0],points_list[i][1],points_list[i][2]))
                if points_list[i][0] == points_spawn_list[i][0]+offesets[i][0] and points_list[i][1] == points_spawn_list[i][1]+offesets[i][1] and points_list[i][2] == points_spawn_list[i][2]+offesets[i][2]:
                    stop = stop +1
                rospy.sleep(1)
        
        points_list = points_spawn_list

        rospy.sleep(TIME_CHANGE)
        for i in range(len(offesets)):
            x = points_spawn_list[i][0]
            y = points_spawn_list[i][1]
            z = points_spawn_list[i][2]
            change_state(name='object'+str(i),new_state=(x,y,z))
        rospy.sleep(TIME_CHANGE)
    ##################################################
