#ifndef ACTUATORCONTROLLER_ROS_H
#define ACTUATORCONTROLLER_ROS_H


#include "ros/ros.h"

// Actuator SDK
#include "actuatorcontroller.h"
#include "actuatordefine.h"


// standard message type
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>



// custom message type
#include "actuatorcontroller_ros/ActuatorAttribute.h"
#include "actuatorcontroller_ros/ActuatorCommand.h"
#include "actuatorcontroller_ros/ActuatorModes.h"
#include "actuatorcontroller_ros/ActuatorArray.h"

// custom service type
#include "actuatorcontroller_ros/AttributeLookup.h"
#include "actuatorcontroller_ros/AttributeQuery.h"
#include "actuatorcontroller_ros/GeneralQuery.h"

// std stuff
#include <map>
#include <string>


class ActuatorController_ROS {

public:


    ActuatorController_ROS();

    ~ActuatorController_ROS();


    void releaseJointStates();

    //Subscriber Callback
    void subscribeChangeAttribute(const actuatorcontroller_ros::ActuatorAttribute & msg);

    void subscribeEnableActuator(const actuatorcontroller_ros::ActuatorArray & msg);

    void subscribeDisableActuator(const actuatorcontroller_ros::ActuatorArray & msg);

    void subscribeSetTargetPosition(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetTargetVelocity(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetTargetCurrent(const actuatorcontroller_ros::ActuatorCommand & msg);

    void subscribeSetControlMode(const actuatorcontroller_ros::ActuatorModes & msg);

    bool serviceAttributeQuery(actuatorcontroller_ros::AttributeQueryRequest & req,
                              actuatorcontroller_ros::AttributeQueryResponse & res );

    bool serviceGeneralQuery(actuatorcontroller_ros::GeneralQueryRequest & req,
                            actuatorcontroller_ros::GeneralQueryResponse & res );

    bool serviceAttributeLookup(actuatorcontroller_ros::AttributeLookupRequest & req,
                               actuatorcontroller_ros::AttributeLookupResponse & res );

private:


    // private handles
	ros::NodeHandle nh;
    ActuatorController *m_pController;


    // publisher for current actuator information
	ros::Publisher m_pubJointState;

	ros::Subscriber m_subAttributeChange;
	ros::Subscriber m_subEnableActuator;
    ros::Subscriber m_subDisableActuator;
	ros::Subscriber m_subTargetPosition;
	ros::Subscriber m_subTargetVelocity;
	ros::Subscriber m_subTargetCurrent;
	ros::Subscriber m_subControlMode;

	
	ros::ServiceServer m_serAttributeQuery;
	ros::ServiceServer m_serGeneralQuery;
	ros::ServiceServer m_serAttributeLookup;


	// temp message




}; //class

#endif 

