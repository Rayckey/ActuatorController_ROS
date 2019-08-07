#include "actuatorcontroller_ros.h"
#include <iostream>


ActuatorController_ROS::ActuatorController_ROS(){

    m_pController = ActuatorController::getInstance();


    m_pubJointState = nh.advertise<sensor_msgs::JointState>("/INNFOS/actuator_states", 10);

    m_subAttributeChange = nh.subscribe("/INNFOS/changeAttribute", 100,
                                              &ActuatorController_ROS::subscribeChangeAttribute, this);


    m_subEnableActuator = nh.subscribe("/INNFOS/enableActuator", 100,
                                              &ActuatorController_ROS::subscribeEnableActuator, this);

    m_subDisableActuator = nh.subscribe("/INNFOS/disableActuator", 100,
                                        &ActuatorController_ROS::subscribeDisableActuator, this);

    m_subTargetPosition = nh.subscribe("/INNFOS/setTargetPosition", 100,
                                              &ActuatorController_ROS::subscribeSetTargetPosition, this);


    m_subTargetVelocity = nh.subscribe("/INNFOS/setTargetVelocity", 100,
                                              &ActuatorController_ROS::subscribeSetTargetVelocity, this);

    m_subTargetCurrent = nh.subscribe("/INNFOS/setTargetCurrent", 100,
                                       &ActuatorController_ROS::subscribeSetTargetCurrent, this);

    m_subControlMode = nh.subscribe("/INNFOS/setControlMode", 100,
                                      &ActuatorController_ROS::subscribeSetControlMode, this);


    m_serAttributeQuery = nh.advertiseService("/INNFOS/AttributeQuery", &ActuatorController_ROS::serviceAttributeQuery, this);
    m_serGeneralQuery  = nh.advertiseService("/INNFOS/GeneralQuery", &ActuatorController_ROS::serviceGeneralQuery, this);
    m_serAttributeLookup = nh.advertiseService("/INNFOS/AttributeLookup", &ActuatorController_ROS::serviceAttributeLookup, this);
}


ActuatorController_ROS::~ActuatorController_ROS() {
    // unregister all publishers here


    m_pubJointState.shutdown();


}




void ActuatorController_ROS::releaseJointStates(){

    std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();


    std_msgs::Header temp_header;
    temp_header.stamp = ros::Time::now();
    sensor_msgs::JointState temp_jointstate;
    temp_jointstate.header = temp_header;


    for (uint8_t joint_id : temp_vec){

        if ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized) {

            temp_jointstate.name.push_back("Actuator" + std::to_string(int(joint_id)));
            temp_jointstate.position.push_back(m_pController->getPosition( joint_id,true));
            temp_jointstate.velocity.push_back(m_pController->getVelocity( joint_id,true));
            temp_jointstate.effort.push_back(m_pController->getCurrent( joint_id,true));

        }
    }

    if (temp_vec.size())
        m_pubJointState.publish(temp_jointstate);

}

//Subscriber Callback
void ActuatorController_ROS::subscribeChangeAttribute(const actuatorcontroller_ros::ActuatorAttribute & msg){
    m_pController->setActuatorAttribute(uint8_t(msg.JointID) , ActuatorAttribute(msg.AttributeID) ,  msg.Value);
}

void ActuatorController_ROS::subscribeEnableActuator(const actuatorcontroller_ros::ActuatorArray & msg){

        std::vector<int> temp_vec;

        int num_of_actuator= msg.JointIDs.size();

        if (num_of_actuator) {
            for (int j = 0 ; j < num_of_actuator ; j ++) {
                m_pController->enableActuator(uint8_t(msg.JointIDs[j]));
            }
        }else {
            ROS_INFO("Turning on all actuators");
            m_pController->enableAllActuators();
        }

}


void ActuatorController_ROS::subscribeDisableActuator(const actuatorcontroller_ros::ActuatorArray & msg){

    std::vector<int> temp_vec;

    int num_of_actuator= msg.JointIDs.size();

    if (num_of_actuator) {
        for (int j = 0 ; j < num_of_actuator ; j ++) {
            m_pController->disableActuator(uint8_t(msg.JointIDs[j]));
        }
    }else {
        ROS_INFO("Turning off all actuators");
        m_pController->disableAllActuators();
    }

}

void ActuatorController_ROS::subscribeSetTargetPosition(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setPosition(uint8_t(msg.JointID) , msg.TargetValue);
}

void ActuatorController_ROS::subscribeSetTargetVelocity(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setVelocity(uint8_t(msg.JointID) , msg.TargetValue);
}

void ActuatorController_ROS::subscribeSetTargetCurrent(const actuatorcontroller_ros::ActuatorCommand & msg){
    m_pController->setCurrent(uint8_t(msg.JointID) , msg.TargetValue);
}

void ActuatorController_ROS::subscribeSetControlMode(const actuatorcontroller_ros::ActuatorModes & msg){

    std::vector<uint8_t> temp_vec;

    int num_of_actuator= msg.JointIDs.size();

    if (num_of_actuator) {
        for (int j = 0 ; j < num_of_actuator ; j ++) {
            temp_vec.push_back(msg.JointIDs[j]);
        }


    }else {
        temp_vec = m_pController->getActuatorIdArray();
    }

    m_pController->activateActuatorModeInBantch(temp_vec , Actuator::ActuatorMode(msg.ActuatorMode));

}

bool ActuatorController_ROS::serviceAttributeQuery(actuatorcontroller_ros::AttributeQueryRequest & req,
                           actuatorcontroller_ros::AttributeQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);

    res.CUR_IQ_SETTING  = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CUR_IQ_SETTING);
    res.CUR_PROPORTIONAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CUR_PROPORTIONAL);
    res.CUR_INTEGRAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CUR_INTEGRAL     );
    res.CUR_ID_SETTING = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CUR_ID_SETTING   );
    res.CUR_MAXSPEED = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CUR_MAXSPEED     );
    res.ACTUAL_CURRENT = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUAL_CURRENT   );
    res.VEL_SETTING = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VEL_SETTING      );
    res.VEL_PROPORTIONAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VEL_PROPORTIONAL );
    res.VEL_INTEGRAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VEL_INTEGRAL     );
    res.VEL_OUTPUT_LIMITATION_MINIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VEL_OUTPUT_LIMITATION_MINIMUM);
    res.VEL_OUTPUT_LIMITATION_MAXIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VEL_OUTPUT_LIMITATION_MAXIMUM);
    res.ACTUAL_VELOCITY = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUAL_VELOCITY);
    res.POS_SETTING = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_SETTING );
    res.POS_PROPORTIONAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_PROPORTIONAL);
    res.POS_INTEGRAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_INTEGRAL);
    res.POS_DIFFERENTIAL = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_DIFFERENTIAL);
    res.POS_OUTPUT_LIMITATION_MINIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_OUTPUT_LIMITATION_MINIMUM);
    res.POS_OUTPUT_LIMITATION_MAXIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_OUTPUT_LIMITATION_MAXIMUM);
    res.POS_LIMITATION_MINIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_LIMITATION_MINIMUM);
    res.POS_LIMITATION_MAXIMUM = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_LIMITATION_MAXIMUM);
    res.HOMING_POSITION = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::HOMING_POSITION);
    res.ACTUAL_POSITION = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUAL_POSITION);
    res.PROFILE_POS_MAX_SPEED = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_POS_MAX_SPEED);
    res.PROFILE_POS_ACC = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_POS_ACC);
    res.PROFILE_POS_DEC = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_POS_DEC);
    res.PROFILE_VEL_MAX_SPEED = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_VEL_MAX_SPEED);
    res.PROFILE_VEL_ACC = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_VEL_ACC);
    res.PROFILE_VEL_DEC = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::PROFILE_VEL_DEC);
    res.POS_OFFSET = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_OFFSET);
    res.VOLTAGE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VOLTAGE);
    res.POS_LIMITATION_SWITCH = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::POS_LIMITATION_SWITCH);
    res.CURRENT_SCALE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CURRENT_SCALE );
    res.VELOCITY_SCALE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VELOCITY_SCALE);
    res.FILTER_C_STATUS = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_C_STATUS);
    res.FILTER_C_VALUE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_C_VALUE );
    res.FILTER_V_STATUS = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_V_STATUS);
    res.FILTER_V_VALUE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_V_VALUE );
    res.FILTER_P_STATUS = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_P_STATUS);
    res.FILTER_P_VALUE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FILTER_P_VALUE );
    res.LOCK_ENERGY = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::LOCK_ENERGY);
    res.ACTUATOR_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUATOR_TEMPERATURE);
    res.INVERTER_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::INVERTER_TEMPERATURE);
    res.ACTUATOR_PROTECT_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUATOR_PROTECT_TEMPERATURE);
    res.ACTUATOR_RECOVERY_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUATOR_RECOVERY_TEMPERATURE);
    res.INVERTER_PROTECT_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::INVERTER_PROTECT_TEMPERATURE);
    res.INVERTER_RECOVERY_TEMPERATURE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::INVERTER_RECOVERY_TEMPERATURE);
    res.ACTUATOR_SWITCH = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ACTUATOR_SWITCH );
    res.FIRMWARE_VERSION = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::FIRMWARE_VERSION);
    res.ONLINE_STATUS = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ONLINE_STATUS);
    res.SN_ID = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::SN_ID   );
    res.MODE_ID = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::MODE_ID );
    res.ERROR_ID = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::ERROR_ID);
    res.CURRENT_LIMIT = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::CURRENT_LIMIT );
    res.VELOCITY_LIMIT = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::VELOCITY_LIMIT);
    res.INIT_STATE = m_pController->getActuatorAttribute(joint_id , ActuatorAttribute::INIT_STATE );


    return true;
}

bool ActuatorController_ROS::serviceGeneralQuery(actuatorcontroller_ros::GeneralQueryRequest & req,
                         actuatorcontroller_ros::GeneralQueryResponse & res ){


    std::vector<uint8_t> temp_vec = m_pController->getActuatorIdArray();

    for (uint8_t joint_id : temp_vec){
        res.ActuatorList.push_back(joint_id);
        res.ActuatorSwitch.push_back((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized);
    }


    return true;

}

bool ActuatorController_ROS::serviceAttributeLookup(actuatorcontroller_ros::AttributeLookupRequest & req,
                            actuatorcontroller_ros::AttributeLookupResponse & res ){


    res.CUR_IQ_SETTING = ActuatorAttribute::CUR_IQ_SETTING;
    res.CUR_PROPORTIONAL = ActuatorAttribute::CUR_PROPORTIONAL;
    res.CUR_INTEGRAL = ActuatorAttribute::CUR_INTEGRAL;
    res.CUR_ID_SETTING = ActuatorAttribute::CUR_ID_SETTING;
    res.CUR_MAXSPEED = ActuatorAttribute::CUR_MAXSPEED;
    res.ACTUAL_CURRENT = ActuatorAttribute::ACTUAL_CURRENT;
    res.VEL_SETTING = ActuatorAttribute::VEL_SETTING;
    res.VEL_PROPORTIONAL = ActuatorAttribute::VEL_PROPORTIONAL;
    res.VEL_INTEGRAL = ActuatorAttribute::VEL_INTEGRAL;
    res.VEL_OUTPUT_LIMITATION_MINIMUM = ActuatorAttribute::VEL_OUTPUT_LIMITATION_MINIMUM;
    res.VEL_OUTPUT_LIMITATION_MAXIMUM = ActuatorAttribute::VEL_OUTPUT_LIMITATION_MAXIMUM;
    res.ACTUAL_VELOCITY = ActuatorAttribute::ACTUAL_VELOCITY;
    res.POS_SETTING = ActuatorAttribute::POS_SETTING;
    res.POS_PROPORTIONAL = ActuatorAttribute::POS_PROPORTIONAL;
    res.POS_INTEGRAL = ActuatorAttribute::POS_INTEGRAL;
    res.POS_DIFFERENTIAL = ActuatorAttribute::POS_DIFFERENTIAL;
    res.POS_OUTPUT_LIMITATION_MINIMUM = ActuatorAttribute::POS_OUTPUT_LIMITATION_MINIMUM;
    res.POS_OUTPUT_LIMITATION_MAXIMUM = ActuatorAttribute::POS_OUTPUT_LIMITATION_MAXIMUM;
    res.POS_LIMITATION_MINIMUM = ActuatorAttribute::POS_LIMITATION_MINIMUM;
    res.POS_LIMITATION_MAXIMUM = ActuatorAttribute::POS_LIMITATION_MAXIMUM;
    res.HOMING_POSITION = ActuatorAttribute::HOMING_POSITION;
    res.ACTUAL_POSITION = ActuatorAttribute::ACTUAL_POSITION;
    res.PROFILE_POS_MAX_SPEED = ActuatorAttribute::PROFILE_POS_MAX_SPEED;
    res.PROFILE_POS_ACC = ActuatorAttribute::PROFILE_POS_ACC;
    res.PROFILE_POS_DEC = ActuatorAttribute::PROFILE_POS_DEC;
    res.PROFILE_VEL_MAX_SPEED = ActuatorAttribute::PROFILE_VEL_MAX_SPEED;
    res.PROFILE_VEL_ACC = ActuatorAttribute::PROFILE_VEL_ACC;
    res.PROFILE_VEL_DEC = ActuatorAttribute::PROFILE_VEL_DEC;
    res.POS_OFFSET = ActuatorAttribute::POS_OFFSET;
    res.VOLTAGE = ActuatorAttribute::VOLTAGE;
    res.POS_LIMITATION_SWITCH = ActuatorAttribute::POS_LIMITATION_SWITCH;
    res.CURRENT_SCALE = ActuatorAttribute::CURRENT_SCALE;
    res.VELOCITY_SCALE = ActuatorAttribute::VELOCITY_SCALE;
    res.FILTER_C_STATUS = ActuatorAttribute::FILTER_C_STATUS;
    res.FILTER_C_VALUE = ActuatorAttribute::FILTER_C_VALUE;
    res.FILTER_V_STATUS = ActuatorAttribute::FILTER_V_STATUS;
    res.FILTER_V_VALUE = ActuatorAttribute::FILTER_V_VALUE;
    res.FILTER_P_STATUS = ActuatorAttribute::FILTER_P_STATUS;
    res.FILTER_P_VALUE = ActuatorAttribute::FILTER_P_VALUE;
    res.LOCK_ENERGY = ActuatorAttribute::LOCK_ENERGY;
    res.ACTUATOR_TEMPERATURE = ActuatorAttribute::ACTUATOR_TEMPERATURE;
    res.INVERTER_TEMPERATURE = ActuatorAttribute::INVERTER_TEMPERATURE;
    res.ACTUATOR_PROTECT_TEMPERATURE = ActuatorAttribute::ACTUATOR_PROTECT_TEMPERATURE;
    res.ACTUATOR_RECOVERY_TEMPERATURE = ActuatorAttribute::ACTUATOR_RECOVERY_TEMPERATURE;
    res.INVERTER_PROTECT_TEMPERATURE = ActuatorAttribute::INVERTER_PROTECT_TEMPERATURE;
    res.INVERTER_RECOVERY_TEMPERATURE = ActuatorAttribute::INVERTER_RECOVERY_TEMPERATURE;
    res.ACTUATOR_SWITCH = ActuatorAttribute::ACTUATOR_SWITCH;
    res.FIRMWARE_VERSION = ActuatorAttribute::FIRMWARE_VERSION;
    res.ONLINE_STATUS = ActuatorAttribute::ONLINE_STATUS;
    res.SN_ID = ActuatorAttribute::SN_ID;
    res.MODE_ID = ActuatorAttribute::MODE_ID;
    res.ERROR_ID = ActuatorAttribute::ERROR_ID;
    res.CURRENT_LIMIT = ActuatorAttribute::CURRENT_LIMIT;
    res.VELOCITY_LIMIT = ActuatorAttribute::VELOCITY_LIMIT;
    res.INIT_STATE = ActuatorAttribute::INIT_STATE;


    return true;

}