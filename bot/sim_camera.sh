#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_cv.launch mode:=0; exec bash'"