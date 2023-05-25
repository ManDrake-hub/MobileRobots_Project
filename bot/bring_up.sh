#! bin/bash
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; source .bashrc; roscore; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; python3 bringup.py; exec bash'"
sleep 5