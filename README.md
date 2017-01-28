The steps to reproduce this project will be explain. The Baxter robot have 2 gripper to move object, also, one camera in each arm thay let us  The project is develop for Ubuntu 14.04 using ROS. 
##0-Dependence
It necessary to get the workspace. In http://sdk.rethinkrobotics.com/wiki/Workstation_Setup are the step to get it. 

##
##2-Robot Conection


Before to execute avahi is recommedable eneable networking in the computer.
```
roscore
```




```
sud avadi-autoipd eth0
```
It will set a IP for the baxter. Go to the baxter-workspace and edit baxter.sh (default name) file. You have to change the variable your_ip. After that, execute.


```
./baxter.sh
```


To prove the connection, we enable the robot
```
rosrun baxter_tools enable_robot.py -e
```




