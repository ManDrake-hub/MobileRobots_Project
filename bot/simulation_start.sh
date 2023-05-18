#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_pyzbar.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch navigation move_simple_goal.launch; exec bash'"

