#! bin/bash
source devel/setup.bash

# Exercise 1
s = Exercise1
printf "&s\n"
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_1.launch 
kill $PID
sleep 30

# Exercise 2
s = Exercise2
printf "&s\n"
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_2.launch 
kill $PID
sleep 30

# Exercise 3
s = Exercise3
printf "&s\n"
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_3.launch 
kill $PID
sleep 30

# Exercise 4
s = Exercise4
printf "&s\n"
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_4.launch 
kill $PID
sleep 30

# Exercise 5
s = Exercise5
printf "&s\n"

roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch robot_pose_ekf robot_pose_ekf.launch & 
PID1=$!
sleep 3
roslaunch navigation exercise_5.launch 
kill $PID1
kill $PID