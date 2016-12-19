The steps to reproduce this project will be explain. The project is develop for Ubuntu 14.04 using ROS. The workspace is explain in previous git.  


##0-Dependence
##1-Robot Conection


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




