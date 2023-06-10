#! /usr/bin/python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose,Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import os 

def spawn_udf_model(name,position):
    try:
        model_path = os.path.dirname(__file__) + "/box.urdf"
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
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service('/gazebo/set_model_state')

    # Parameters
    time_to_reach_end = 3.0
    time_step = 0.01
    rate = rospy.Rate(1 / time_step)
    points_spawn_list = [(5,0,0),(15.7,-5,0),(15.2,-2.5,0),(8.25,-9.5,0),(2.2,-10.9,0),(-4.75,-8.35,0),(24.5,-9.25,0),(36.5,-8.4,0),(32.4,-1.05,0),(21.85,-1.4,0)]
    points_end_list = [(8,0.15,0),(15.7,-8,0),(15.2,-8.5,0),(4.25,-9.5,0),(1.2,-9.9,0),(-4.75,-4.35,0),(20.5,-10.25,0),(35.5,-4.4,0),(28.4,-1.05,0),(20.85,-0.4,0)]
    
    # Obstacle spawning
    for i in range(len(points_spawn_list)):
        spawn_udf_model(name='object'+str(i),position=points_spawn_list[i])
    rospy.sleep(3)

    # Position changing
    move_factor = 0.0
    while not rospy.is_shutdown():
        time_spent = 0.0
        while time_spent < time_to_reach_end:
            move_factor = time_spent / time_to_reach_end
            i = 0
            for position_start, position_end in zip(points_spawn_list, points_end_list):
                point = ((1.0 - move_factor) * position_start[0] + move_factor*position_end[0],
                         (1.0 - move_factor) * position_start[1] + move_factor*position_end[1],
                         (1.0 - move_factor) * position_start[2] + move_factor*position_end[2])
                change_state(name='object'+str(i), new_state=point)
                i += 1
            time_spent += time_step
            rate.sleep()

        _points_spawn_list = points_spawn_list
        points_spawn_list = points_end_list
        points_end_list = _points_spawn_list
