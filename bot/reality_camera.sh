#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; python3 src/realsense/launch/launch.py; exec bash'"
sleep 15
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_cv.launch mode:=1; exec bash'"