# MobileRobots_Project

## Authors
Course: Mobile Robots for critical missions 2022/2023
 
Lecturers: 
* Vento Mario	       mvento@unisa.it
* Saggese Alessia    asaggese@unisa.it
 
Group:
* Mandragora Geraldino 0622701875    g.mandragora@studenti.unisa.it
* Sullutrone Giovanni  0622701751    g.sullutrone2@studenti.unisa.it
* Tirino Francesca     0622701745    f.tirino@studenti.unisa.it
* Farina Luigi         0622701754    l.farina16@studenti.unisa.it

## Application Demo
* WaypointMovement class manage the control of the movement of the robot. You can set the waypoints for the robot's movement using the set_waypoints() method. This method takes a list of numpy arrays, where each numpy array represents a waypoint in the form [x, y, theta]. The theta value represents the angle at which the robot should be facing when it reaches the waypoint.
You can then start the robot's movement by calling the play() method that move the robot to each waypoint in the list in order, stopping at each waypoint and rotating to face the next waypoint before continuing.
* KalmanFilter class implements a custom solution of it as uniformly accelerated motion.
* ExtendedKalmanFilter uses robot_pose_ekf node.
* OdomNoiser class subscribes to the odom topic and adds noise to the odometry data. It publishes the noisy odometry data to the odom_noised topic.

For each exercise, we have previously run 20 tests to check the variance and the std deviation obtained. To modify this value, it is necessary to modify RUNS variable in each exercise.py files. 
To avoid slipping, we implemented functionalities that let us increase/decrease gradually our speed (i.e. we calculate the space needed to come to a full stop with the current speed and decrease or increase the speed accordingly to reach the requested waypoint in the smoothest way possible).

For each exercise we will include an image that shows of each (numbered) waypoint, its position and the area at one std from it.

* Exercise 1:
Apply the command (a-priori known) for reaching the next waypoint. 
<img src="/bot/src/navigation/images/Figure_1.png"/>

* Exercise 2:
The movement model is affected by gaussian noise characterized by 0 mean and 0.25 std.
<img src="/bot/src/navigation/images/Figure_2.png"/>

* Exercise 3:
The movement model and measurement model are affected by gaussian noise characterized by 0 mean and 0.25 std. 
<img src="/bot/src/navigation/images/Figure_3.png"/>

* Exercise 4:
We use a custom Kalman filter for a straight path. It is uniformly accelerated because we modify the speed during the movement. As the initial state, we consider the robot's initial position (0,0,0). Consequently, the update is done through the measurements obtained from the laser scan. In the below figure, the std of the fourth waypoint represent the fact that the robot hits the wall.
<img src="/bot/src/navigation/images/Figure_4.png"/>

* Exercise 5:
The already implemented ROS ekf node is used to set the robot's belief. Again, the measurement model is affected by gaussian noise characterized by 0 mean and 0.25 std. For this reason, it was necessary to remap the odom topic in the .launch file.
<img src="/bot/src/navigation/images/Figure_5.png"/>

## Install
Install ros as done during the lectures 

In case of errors:
* Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python3-empy'

  * ```pip install empy ```

* ImportError: "from catkin_pkg.package import parse_package" failed: No module named 'catkin_pkg'

  * ```pip install catkin-pkg``` 
  
* CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:463 (message): A required package was not found

  * ```sudo apt-get install ros-noetic-robot-pose-ekf``` 

* ModuleNotFoundError: No module named 'yaml'

  * ```pip install pyyaml``` 

* Substitution args not supported:  No module named 'rospkg'

  * ```pip install rospkg``` 

* Substitution args not supported:  No module named 'defusedxml'

  * ```pip install defusedxml``` 

* ModuleNotFoundError: No module named 'numpy'

  * ```pip install numpy``` 


## How to use
Follow these steps:
* cd into the folder MobileRobots_Project/bot
* ```chmod u+x src/navigation/src/*``` 
* ```catkin build``` 
* ```bash terminals.sh``` 

The "terminals.sh" command will start one simulation for all 5 exercises.

## License
[GNU](https://choosealicense.com/licenses/gpl-3.0/)
