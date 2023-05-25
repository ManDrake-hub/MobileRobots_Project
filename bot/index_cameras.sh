#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; python3 src/realsense/launch/launch.py --query; exec bash'"
sleep 20
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; rosparam get /camera/indexes; exec bash'"
