#! bin/bash
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roscore; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch turtlebot3_diem_sim turtlebot3_diem_sim.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch navigation clear_costmap.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; roslaunch navigation obstacle_race.launch; exec bash'"
sleep 3
gnome-terminal --tab -e "bash -c 'source devel/setup.bash; rosrun rqt_reconfigure rqt_reconfigure; exec bash'"
#sleep 3
#gnome-terminal --tab -e "bash -c 'source devel/setup.bash; rqt_graph; exec bash'"
#sleep 3
#gnome-terminal --tab -e "bash -c 'source devel/setup.bash; rosparam set enable_statistics true; exec bash'"
