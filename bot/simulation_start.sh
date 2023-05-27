#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch navigation move_simple_goal.launch; exec bash'"