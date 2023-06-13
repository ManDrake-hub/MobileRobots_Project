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
    points_spawn_list = [(5,0,0),(15.7,-5,0),(-4.75,-8.35,0),(21.85,-1.4,0)]
    points_end_list = [(8,0.15,0),(15.7,-8,0),(-4.75,-4.35,0),(20.85,-0.4,0)]
    
    points = [(-6.45,-6,0),(-3,-5.6,0),(17.15,-6.4,0),(13.9,-6.4,0),(3.05,-0.5,0),(29.65,-1.1,0),(34.65,-7,0),(37.05,-7.1,0),
              (6.0,0,0),(24.0,0,0),(-5.13,-7.69,0),(1.70,-9.32,0),(5.53,-9.99,0),(8.45,-9.19,0),(15.02,-3.16,0),(15.01,-5.57,0),
              (14.92,-8.27,0),(23.77,-9.89,0),(28.47,-10.08,0),(30.59,-10.79,0),(35.74,-3.67,0),(35.73,-6.46,0),(35.61,-9.28,0)]

    # Obstacle spawning
    for i in range(len(points)):
        spawn_udf_model(name='static'+str(i),position=points[i])
        rospy.sleep(0.1)
    
    for i in range(len(points_spawn_list)):
        spawn_udf_model(name='object'+str(i),position=points_spawn_list[i])
        rospy.sleep(0.1)

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
