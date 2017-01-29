The steps to reproduce this project will be explain. The Baxter robot has two gripper, also, one camera in each arm thay let us manipulation of some objects. In this work, we are working to develop the object clasification using open CV and inverse kinematic. The project is develop for Ubuntu 14.04 using ROS. 

##0-Dependence
###Workspace
It necessary to get the defult workspace for the Baxter. In http://sdk.rethinkrobotics.com/wiki/Workstation_Setup are the step to get it. 
###Libraries
In source_lib directory there is a osik-full.zip file. You must decompress it. After that:
```
cd ext
./install-ext   ~/devel/install -a
cd ...
mkdir build
cd build
cmake ..
make
```
##1.-Ros Packages
This git contain the files neccesary to work as another packages. You could edit the "CMakeLists.txt" file to modify the name of the project. After that you could compile the whole workspace.
```
roscd 
cd ..
catkin_make
```

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




