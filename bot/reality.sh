#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map/map.yaml; exec bash'"