#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/map/map.yaml; exec bash'"
sleep 10
