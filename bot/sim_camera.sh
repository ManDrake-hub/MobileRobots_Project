#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_cv_sim.launch; exec bash'"