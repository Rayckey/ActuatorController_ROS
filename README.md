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
Function: Allows users to lookup all avaliable actuators and their status.
Input: A placeholder variable, not needed <br>
Output: Provide a list of all avaliable actuators and their status.<br>

##### /INNFOS/AttributeQuery (ActuatorController_ROS::AttributeQuery)
Function: Allow users to lookup some frequently used modifiable parameters of the actuators. To modify these parameters, please use the designaed servies/messages <br>
Input: The designated actuator ID. <br>
Output: Return a list of the frequently used parameters for the designated actuator.<br>

##### /INNFOS/TriviaQuery (ActuatorController_ROS::TriviaQuery)
Function: Allow users to lookup some frequently used unmodifiable parameters of the actuators. <br>
Input: The designated actuator ID. <br>
Output: Return a list of the frequently used unmodifiable parameters for the designated actuator.<br>

##### /INNFOS/DebugQuery (ActuatorController_ROS::DebugQuery) 
Function: Allow users to lookup some infrequently used unmodifiable parameters of the actuators. Th
Input: The designated actuator ID. <br>
Output: Return a list of the infrequently used unmodifiable parameters for the designated actuator. Only for debugging. <br>

##### /INNFOS/Dictionary (ActuatorController_ROS::AttributeDictionary)
Function: Allows user to look up Attribute terms' meanings and usages.
Input: The attribute term (i.e. "MODE_ID") in string <br>
Output: The explaination and usage of the term or parameter. <br>

##### /INNFOS/IDChange (ActuatorController_ROS::IDModify)
Function: Permentaly download the user's setting into the actuator, allows it to take effect the next time it is powered up. <br>
Input: The original ID, and the modified ID. <br>
Output: Will return a boolean to indicate whether this operation is successful. <br>

##### /INNFOS/ParametersSave (ActuatorController_ROS::ParametersSave)
Function: Permentaly download the user's setting into the actuator, allows it to take effect the next time it is powered up. <br>
Input: The designated actuator ID. <br>
Output: Will return a boolean to indicate whether this operation is successful. <br>


##### /INNFOS/ZeroReset (ActuatorController_ROS::ZeroReset)
Function: Reset the actuator's abosulte zero position to its current position, the actuator has to be in homeing mode for it to take effects. (Due to some issues, the actuator may appear to be in current mode even after setting it to homing mode, but it will still have taken effects.) <br>
Input: The designated actuator ID. <br>
Output: Allows the user to reset the zero position of the actuator. Note that any changes will need to be saved using the /INNFOS/ParametersSave for it to take effects in the next boot-up. <br>


### Parameters Server
This allows the user to modifiy the infrequently used modifiable parameters.<br>
Since each actuator has a number of modifiable parameters, the parameter names on the server are arraged in the format of : <br>
```
/INNFOS/Actuator/${ACTUATOR_ID}/${PARAMETER_NAME}
```
You can look up the parameter using the service /INNFOS/Dictionary. <br>
ote that any changes will need to be saved using the /INNFOS/ParametersSave for it to take effects in the next boot-up. <br>
If the assignment was unsuccessful, the parameter will be reverted on the server.


# Usage Example
For a first time user, it is recommanded to read through the INNFOS wiki page for the setup: https://innfos.github.io/wiki/en/#!index.md <br>
You can start the ROS node as soon as the device is connected to the commnunication bridge. <br>
To ensure all the actuators are connected, you should first check the lists of avaliable actuators via the /INNFOS/GeneralQuery service:
```
rosservice call /INNFOS/GeneralQuery "isQuery: true" 
```
And then, you can enable the actuators of your choice. Here we try to enable all the actuators in one go by using the ID 0:
```
rostopic pub -1 /INNFOS/enableActuator actuatorcontroller_ros/ActuatorArray "JointIDs:
- 0" 

``` 
It should be noted that there should not be any actuators with the ID 0. <br>
By default, the actuators when first launched will be in current mode, you can varify it using the service:
```
rosservice call /INNFOS/AttributeQuery "ActuatorID: 1"
``` 
The "MODE_ID" variable is the active mode of the actuator. <br>
If you come across a term that you need more clarifications of, you can call the service:
```
rosservice call /INNFOS/Dictionary "LookupTerm:
  data: 'MODE_ID'" 
```
You will get a detailed explanation for this parameter.<br>

To enforce position control to the robot, you should first change its mode:
```
rostopic pub -1 /INNFOS/setControlMode actuatorcontroller_ros/ActuatorModes "JointIDs:
- 3
ActuatorMode: 4" 
``` 
Here we changed the third actuator to Mode_Profile_Pos, which allows us to control the actuator's position with local planner. <br>
```
rostopic pub -1 /INNFOS/setTargetPosition actuatorcontroller_ros/ActuatorCommand "JointID: 0
TargetValue: 0.0"
```
And the target position commands can finally take effect.

# Change logs
Added a usage example: 2019/08/21



<table style="width:500px"><thead><tr style="background:PaleTurquoise"><th style="width:100px">Version number</th><th style="width:150px">Update time</th><th style="width:3800px">Update content</th></tr></thead><tbody><tr><td>v1.0.2</td><td>2019.08.21</td><td> Included an example in readme </th></tr></thead><tbody><tr><td>v1.0.1</td><td>2019.08.09</td><td>Added readme</th></tr></thead><tbody><tr><td>v1.0.0</td><td>2019.08.09</td><td>Node tested with actuators on Ubuntu 16.04 with ROS Luna, Stable release</td></tbody></table>
