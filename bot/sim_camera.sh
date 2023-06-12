#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch camera qr_cv.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch realsense realsense_test.launch; exec bash'"
