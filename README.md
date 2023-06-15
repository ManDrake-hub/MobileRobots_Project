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
<img src="/bot/src/navigation/images/Figure_1.png"/>

* Exercise 2:


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
