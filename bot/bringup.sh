#! bin/bash
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; bash install.bash wlo1; cd && source .bashrc; roscore; exec bash'"
sleep 15
gnome-terminal --tab -e "bash -c 'cd $HOME/turtlebot3_network; python3 bringup.py; exec bash'"