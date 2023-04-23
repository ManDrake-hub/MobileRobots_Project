#! bin/bash
source devel/setup.bash

# Exercise 1
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!

sleep 3

roslaunch navigation exercise_1.launch 
kill $PID

# Exercise 2
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_2.launch 
kill $PID

# Exercise 3
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
PID=$!

sleep 3

roslaunch navigation exercise_3.launch 
kill $PID
