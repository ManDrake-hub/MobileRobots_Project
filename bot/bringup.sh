#! bin/bash
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; cd && source .bashrc; roscore; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; python3 bringup.py; exec bash'"
sleep 5
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; python3 src/realsense/launch/launch.py --query; exec bash'"
sleep 15