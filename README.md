# ActuatorController_ROS
A standalone ROS package exclusively for INNFOS actuators
A ROS wrapper for INNFOS Actuator Controller





# How to Install
## ROS
If you have not already, follow the instrutions on http://wiki.ros.org/ to install ROS version of your choice.

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
To run the ros node seperately and start actuators with the default options
```
$ roscore
$ rosrun actuatorcontroller_ros innfos_actuator
```

## ROS launch
You can launch the node with custom parameters if you wish to change the performance of the controller. These launch files are mere examples and their effects can be combined.
```
$ roslaunch actuatorcontroller_ros innfos_no_param.launch
```
This launch file adds the parameter "innfos_no_param" in parameter server. This option will prevent the node from polluting the parameter server when there are too many actuators connected, or prevent other users from altering these parameters.
```
$ roslaunch actuatorcontroller_ros innfos_fixed_100hz.launch
```
This launch file adds the parameter "innfos_fixed_rate" in parameter server.
When this parameter is present, the node will attempt to process the messages at a fixed rate. This allows for higher control rate or lower cpu usage. Please be informed that the actual control rate will be dependent on the equipment setup. 
```
$ roslaunch actuatorcontroller_ros innfos_use_cvp.launch
```
This launch file adds the parameter "innfos_use_cvp" in parameter server.
When this parameter is true, the controller will use a more efficient method when requesting the current states of the actuators. BUT the values will have a slight delay depending on the control rate. This is best used with the "innfos_fixed_rate" parameter.


# ROS Messages, Services & Parameters
## Parameters of an INNFOS actuator
The paramters of each actuator are seperated into four groups: <br>
1.Frequently used modifiable parameters<br>
2.Frequently used unmodifiable parameters<br>
3.Infrequently used modifiable parameters<br>
4.Infrequently used unmodifiable parameters<br>
The most accessed parameters can be inquired or modified using ROS messages and services, the less used parameters are only accesable through the ROS Parameter server. The unmodifiable information can only be viewed using ROS services
You can easily see the messages/services typies using:
```
$ rostopic type ${TOPIC_NAME}
$ rosservice type ${SERVICE_NAME}
```

### Published Topics
##### /INNFOS/actuator_states (sensor_msgs::JointState)
Provide all avaliable actuators' positions, velocities and efforts when the actuators are enabled. Their units are "Rotations", "RPM", and "Amp" respectively. <br>

### Subscribed Topics
##### /INNFOS/enableActuator (ActuatorController_ROS::ActuatorArray)
Enable the designated actuators, if the input is empty or 0, the node will enable all avaliable actuators. <br>

##### /INNFOS/disableActuator (ActuatorController_ROS::ActuatorArray)
Disable the designated actuators, if the input is empty or 0, the node  will disable all avaliable actuators. <br>

##### /INNFOS/setControlMode (ActuatorController_ROS::ActuatorModes)
Set the control mode for the designated actuators, you can check the avaliable modes by using the service /INNFOS/Dictionary. <br>

##### /INNFOS/setTargetPosition (ActuatorController_ROS::ActuatorCommand)
Set the target position for the designated actuator, will only have effects when the actuator is in the correct mode. <br>


##### /INNFOS/setTargetVelocity (ActuatorController_ROS::ActuatorCommand)
Set the target velocity for the designated actuator, will only have effects when the actuator is in the correct mode. <br>


##### /INNFOS/setTargetCurrent (ActuatorController_ROS::ActuatorCommand)
Set the target current for the designated actuator, will only have effects when the actuator is in the correct mode. <br>


### Services
##### /INNFOS/GeneralQuery (ActuatorController_ROS::GeneralQuery)
Provide a list of all avaliable actuators and inform the users if they are enabled.<br>

##### /INNFOS/AttributeQuery (ActuatorController_ROS::AttributeQuery)
Return a list of the frequently used parameters for the designated actuator.<br>

##### /INNFOS/TriviaQuery (ActuatorController_ROS::TriviaQuery)
Return a list of the frequently used unmodifiable parameters for the designated actuator.<br>

##### /INNFOS/DebugQuery (ActuatorController_ROS::DebugQuery) 
Return a list of the infrequently used unmodifiable parameters for the designated actuator. Only for debugging. <br>

##### /INNFOS/Dictionary (ActuatorController_ROS::AttributeDictionary)
Allows user to look up the explaination and usage of an term or parameter. <br>

##### /INNFOS/IDChange (ActuatorController_ROS::IDModify)
Allows user to change the ID of an actuator. Note that any changes will need to be saved using the /INNFOS/ParametersSave for it to take effects in the next boot-up. <br>

##### /INNFOS/ParametersSave (ActuatorController_ROS::ParametersSave)
Permentaly download the user's setting into the actuator, allows it to take effect the next time it is powered up.<br>

##### /INNFOS/ZeroReset (ActuatorController_ROS::ZeroReset)
Allows the user to reset the zero position of the actuator. Note that any changes will need to be saved using the /INNFOS/ParametersSave for it to take effects in the next boot-up. <br>


### Parameters Server
This allows the user to modifiy the infrequently used modifiable parameters.<br>
Since each actuator has a number of modifiable parameters, the parameter names on the server are arraged in the format of : <br>
```
/INNFOS/Actuator/${ACTUATOR_ID}/${PARAMETER_NAME}
```
You can look up the parameter using the service /INNFOS/Dictionary. <br>
ote that any changes will need to be saved using the /INNFOS/ParametersSave for it to take effects in the next boot-up. <br>
If the assignment was unsuccessful, the parameter will be reverted on the server.


