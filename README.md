# ActuatorController_ROS
A standalone ROS package made exclusively for INNFOS actuators





# How to Install
## ROS
If you have not already, follow the instrutions on http://wiki.ros.org/ to install ROS.

## Install
This package is tested on an X86 system with Ubuntu 16.04 installed.

Enter your catkin workspace's source directory.
You can clone the package with:
```
$ git clone https://github.com/Rayckey/ActuatorController_ROS.git actuatorcontroller_ros
```
Then return to the root of your workspace and build the package
```
$ catkin_make
```
There's a chance that the package may fail on the first time it is built, if that happens, retry it couple times.
```
$ source devel/setup.bash
```
Now you're ready to use the package!


# How to Use
## ROS node
To run the ros node seperately
```
$ roscore
$ rosrun actuatorcontroller_ros innfos_actuator
```

## ROS launch
TODO


# ROS Messages, Services & Parameters
## Parameters of an INNFOS actuator
To understand how the messages are distributed, we need to look at the actuator paramters, which I had divided into four categories:
1.Frequently used modifiable parameters
2.Frequently used unmodifiable parameters
3.Infrequently used modifiable parameters
4.Infrequently used unmodifiable parameters
You can easily see the messages/services typies using:
```
$ rostopic type ${TOPIC_NAME}
$ rosservice type ${SERVICE_NAME}
```

### Frequently used modifiable parameters
Most of these parameters can be set thru messages and servies
The parameters deemed mostly used have been made more accessible thru ROS message, they include:
1. A general inquery about the actuators
Using the service /INNFOS/GeneralQuery you can retrive a list of all avaliable actuators, and their status
2. Enable or disable actuators
Send a message thru the /INNFOS/enableActuator or /INNFOS/disableActuator to enable and disable the actuators, respectively.
The message accepts an int32[] array, if the data size is 0 or an ID is 0, it will enable/disable all the actuators
3.



### Frequently used unmodifiable parameters (ROS Messages)
1. The current states of the actuators, including position, velocity and current
You can get all these values by subscribing to the /INNFOS/actuator_states topic, which has the type sensor_msgs/joint_states


