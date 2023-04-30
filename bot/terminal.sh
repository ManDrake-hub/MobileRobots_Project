#! bin/bash
source devel/setup.bash

# Exercise 1
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_1.launch 
kill $PID
sleep 30

# Exercise 2
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_2.launch 
kill $PID
sleep 30

# Exercise 3
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation odom_noised.launch & 
PID1=$!
sleep 3
roslaunch navigation exercise_3.launch 
kill $PID1
kill $PID
sleep 30

# Exercise 4
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation exercise_4.launch 
kill $PID
sleep 30

# Exercise 5
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch & 
PID=$!
sleep 3
roslaunch navigation odom_noised.launch & 
PID1=$!
sleep 3
roslaunch robot_pose_ekf robot_pose_ekf.launch & 
PID2=$!
sleep 3
roslaunch navigation exercise_5.launch 
kill $PID2
kill $PID1
kill $PID
