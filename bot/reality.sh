#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/francesca/Scrivania/MobileRobots_Project/bot/src/map2gazebo/map/map_new.yaml; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch navigation clear_costmap.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; rosrun rqt_reconfigure rqt_reconfigure; exec bash'"
