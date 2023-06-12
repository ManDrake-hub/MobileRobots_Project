#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_cv.launch; exec bash'"
sleep 5
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; python3 src/realsense/launch/launch.py; exec bash'"

