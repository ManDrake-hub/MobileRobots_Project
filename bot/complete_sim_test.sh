#! bin/bash
gnome-terminal --tab -e "bash -c 'sh simulation.sh; exec bash'"
sleep 15
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_test.launch; exec bash'"
sleep 5
gnome-terminal --tab -e "bash -c 'sh simulation_start.sh; exec bash'"
