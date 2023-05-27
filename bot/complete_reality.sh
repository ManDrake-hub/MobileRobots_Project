#! bin/bash
gnome-terminal --tab -e "bash -c 'sh reality.sh; exec bash'"
sleep 15
gnome-terminal --tab -e "bash -c 'sh reality_camera.sh; exec bash'"
sleep 20
gnome-terminal --tab -e "bash -c 'sh reality_start.sh; exec bash'"
