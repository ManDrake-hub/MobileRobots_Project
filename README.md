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
[VIDEO]

* Waypoint to reach:
<img src="/waypoints.png"/>

* Used map
<img src="/map.jpg"/>

* Simulation
<img src="/obstacle_race_gazebo.jpg"/>

## Install
Install ros as done during the lectures 

In case of errors:
* Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python3-empy'

  * ```pip install empy ```

## How to use
Follow these steps:
* cd into the folder MobileRobots_Project/bot
* ```chmod u+x src/navigation/src/*``` 
* ```catkin build``` 
* if you didn't want to use the cameras, ```sh complete_sim_test.sh```
* if you want to use cameras, ```sh complete_simulation.sh``` 

The "sh complete_sim_test.sh" command will start the simulation. There is a terminal where you can write the command to simulate cameras. There is a terminal where you can simulate the kidnapped robot.
The "sh complete_simulation.sh" command will start the simulation. There is a terminal where you can simulate the kidnapped robot.

## Dependencies

to run 
```console
sh terminals.sh 
```
you'll need to install gnome-terminal.<br />
on Ubuntu gnome terminal is installed by default, otherwise you'll need to install it with the following command:<br />

```console
sudo apt-get install gnome-terminal
```

## License
[GNU](https://choosealicense.com/licenses/gpl-3.0/)
