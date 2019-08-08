#include "actuatorcontroller_ros.h"
#include <iostream>




bool ActuatorController_ROS::serviceAttributeQuery(actuatorcontroller_ros::AttributeQueryRequest & req,
                                                   actuatorcontroller_ros::AttributeQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);



    res.ACTUAL_CURRENT = m_pController->getCurrent(joint_id,false);
    res.ACTUAL_VELOCITY = m_pController->getVelocity(joint_id,false);
    res.ACTUAL_POSITION = m_pController->getPosition(joint_id,false);
    res.VOLTAGE = m_pController->getVoltage(joint_id,true);
    res.CURRENT_SCALE = m_pController->getCurrentRange(joint_id,false);
    res.VELOCITY_SCALE = m_pController->getVelocityRange(joint_id,false);
    res.ACTUATOR_TEMPERATURE = m_pController->getMotorTemperature(joint_id,true);
    res.INVERTER_TEMPERATURE = m_pController->getInverterTemperature(joint_id,true);
    res.ACTUATOR_SWITCH = m_pController->isEnable(joint_id);
    res.ONLINE_STATUS = m_pController->isOnline(joint_id);
    res.INIT_STATE = ((int) m_pController->getActuatorAttribute(joint_id, Actuator::INIT_STATE) == Actuator::Initialized));


    return true;
}



bool ActuatorController_ROS::serviceDebugQuery(actuatorcontroller_ros::DebugQueryRequest & req,
                                                   actuatorcontroller_ros::DebugQueryResponse & res ){

    uint8_t joint_id = uint8_t(req.ActuatorID);

    res.FIRMWARE_VERSION =  m_pController->getActuatorAttribute(joint_id, Actuator::FIRMWARE_VERSION);
    res.SN_ID =  m_pController->getActuatorAttribute(joint_id, Actuator::SN_ID);
    res.ERROR_ID = m_pController->getActuatorAttribute(joint_id, Actuator::ERROR_ID);
    res.VERSION_430 = m_pController->getActuatorAttribute(joint_id, Actuator::VERSION_430);
    res.FRENQUENCY_430 = m_pController->getActuatorAttribute(joint_id, Actuator::FRENQUENCY_430);



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




bool ActuatorController_ROS::serviceAttributeDictionary(actuatorcontroller_ros::AttributeDictionaryRequest & req,
                                actuatorcontroller_ros::AttributeDictionaryResponse & res ){

    if (m_mAttributeDictionary.find(req.LookupWord.data)!= m_mAttributeDictionary.end()){

        res.Explanation.data = m_mAttributeDictionary[req.LookupWord.data];
    }else {
        res.Explanation.data = "Invalid term!";
    }


    return true;
}


void ActuatorController_ROS::populateDictionary(){

    m_mAttributeDictionary["CUR_PROPORTIONAL"] = "Type: Double. \nDescription: Proportional value for current mode's "
                                                 "PID control";
    m_mAttributeDictionary["CUR_INTEGRAL"] = "Integral value for current mode's PID control";
    m_mAttributeDictionary["CUR_MAXSPEED"] = "Maximum speed allowed for current mode";
    m_mAttributeDictionary["VEL_PROPORTIONAL"] = "Proportional value for velocity mode's PID control";
    m_mAttributeDictionary["VEL_INTEGRAL"] = "Integral value for velocity mode's PID control";
    m_mAttributeDictionary["VEL_OUTPUT_LIMITATION_MINIMUM"] = "Maximum (in negative direction) ratio of the physical "
                                                              "current limit allowed in velocity mode, valid from "
                                                              "-1.0 to 0.0, if the physical current limit is 33A and "
                                                              "this value is -0.5, then the maximum current "
                                                              "(in negative direction) output in "
                                                              "velocity mode is -16.5A";
    m_mAttributeDictionary["VEL_OUTPUT_LIMITATION_MAXIMUM"] = "Maximum ratio of the physical current limit allowed "
                                                              "in velocity mode, valid from "
                                                              "0.0 to 1.0, if the physical current limit is 33A and "
                                                              "this value is 0.5, then the maximum current output in "
                                                              "velocity mode is 16.5A";
    m_mAttributeDictionary["POS_PROPORTIONAL"] = "Proportional value for position mode's PID control";
    m_mAttributeDictionary["POS_INTEGRAL"] = "Integral value for position mode's PID control";
    m_mAttributeDictionary["POS_DIFFERENTIAL"] = "Differential value for position mode's PID control";
    m_mAttributeDictionary["POS_OUTPUT_LIMITATION_MINIMUM"] = "Maximum (in negative direction) ratio of the physical "
                                                              "current limit allowed in position mode, valid from "
                                                              "-1.0 to 0.0, if the physical current limit is 33A and "
                                                              "this value is -0.5, then the maximum current "
                                                              "(in negative direction) output in "
                                                              "position mode is -16.5A";
    m_mAttributeDictionary["POS_OUTPUT_LIMITATION_MAXIMUM"] = "Maximum ratio of the physical current limit allowed "
                                                              "in position mode, valid from "
                                                              "0.0 to 1.0, if the physical current limit is 33A and "
                                                              "this value is 0.5, then the maximum current output in "
                                                              "position mode is 16.5A";
    m_mAttributeDictionary["POS_LIMITATION_MINIMUM"] = "Lower limit for position mode, any target value lower than "
                                                       "this limit will be replace by this value";
    m_mAttributeDictionary["POS_LIMITATION_MAXIMUM"] = "Upper limit for position mode, any target value higher than "
                                                       "this limit will be replace by this value";
    m_mAttributeDictionary["PROFILE_POS_MAX_SPEED"] = "The maximum speed allowed in the local planner for profile "
                                                      "position mode";
    m_mAttributeDictionary["PROFILE_POS_ACC"] = "The acceleration in the local planner for profile position mode, "
                                                "used when actuator is approaching maximum speed";
    m_mAttributeDictionary["PROFILE_POS_DEC"] = "The deceleration in the local planner for profile position mode, "
                                                "used when actuator is coming to a stop ";
    m_mAttributeDictionary["PROFILE_VEL_MAX_SPEED"] = "The maximum speed allowed in the local planner for profile "
                                                      "velocity mode";
    m_mAttributeDictionary["PROFILE_VEL_ACC"] = "The acceleration in the local planner for profile velocity mode, "
                                                "used when actuator is approaching maximum speed";
    m_mAttributeDictionary["PROFILE_VEL_DEC"] = "The deceleration in the local planner for profile velocity mode, "
                                                "used when actuator is coming to a stop ";
    m_mAttributeDictionary["POS_LIMITATION_SWITCH"] = "Determine if the maximum and minimum position limits will "
                                                      "be in effect in velocity/current mode";
    m_mAttributeDictionary["FILTER_C_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target currents";
    m_mAttributeDictionary["FILTER_C_VALUE"] = "The filter cut-off value of the target currents";
    m_mAttributeDictionary["FILTER_V_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target velocities";
    m_mAttributeDictionary["FILTER_V_VALUE"] = "The filter cut-off value of the target velocities";
    m_mAttributeDictionary["FILTER_P_STATUS"] = "Determine if a first order low pass filter will be used for the "
                                                "target positions";
    m_mAttributeDictionary["FILTER_P_VALUE"] = "The filter cut-off value of the target positions";
    m_mAttributeDictionary["LOCK_ENERGY"] = "The maximum amount of energy allowed in an deadlock situation, "
                                            "if the actuator had spent this much energy but cannot progress, "
                                            "the target output will be gradually lowered";
    m_mAttributeDictionary["ACTUATOR_PROTECT_TEMPERATURE"] = "The maximum actuator temperature allowed during operation,"
                                                             " should the actuator reach this temperature, "
                                                             "it will enter a lock down mode and reject most commands";
    m_mAttributeDictionary["ACTUATOR_RECOVERY_TEMPERATURE"] = "The recovery temperature of the actuator, if the "
                                                              "actuator is in lock down mode and is lowered to this "
                                                              "temperature, it will resume its functions";
    m_mAttributeDictionary["INVERTER_PROTECT_TEMPERATURE"] = "The maximum inverter temperature allowed during operation,"
                                                             " should the actuator reach this temperature, "
                                                             "it will enter a lock down mode and reject most commands";
    m_mAttributeDictionary["INVERTER_RECOVERY_TEMPERATURE"] = "The recovery temperature of the inverter, if the "
                                                              "actuator is in lock down mode and is lowered to this "
                                                              "temperature, it will resume its functions";
    m_mAttributeDictionary["CURRENT_LIMIT"] = "A software limit on the current outputs in current mode, any"
                                              " target exceeding this value will be replaced by this value";
    m_mAttributeDictionary["VELOCITY_LIMIT"] = "A software limit on the velocity outputs in velocity mode, any"
                                               " target exceeding this value will be replaced by this value";
    m_mAttributeDictionary["MODE_ID"] = "The control mode of the actuator currently in effect. "
                                        "Options include: Mode_Cur, Mode_Vel, Mode_Profile_Pos, "
                                        "Mode_Profile_Vel, Mode_Homing";


    m_mAttributeDictionary["POS_OFFSET"] = "The leeway for velocity/current mode to react when then are approaching "
                                           "the position limits, allowing them to reduce output before exceeding the limit";

    m_mAttributeDictionary["DEVICE_ID"] = "The actuator ID, changeable";




    m_mAttributeDictionary["ACTUAL_CURRENT"] = "The current current of the actuator";
    m_mAttributeDictionary["ACTUAL_VELOCITY"] = "The current velocity of the actuator";
    m_mAttributeDictionary["ACTUAL_POSITION"] = "The current position of the actuator";
    m_mAttributeDictionary["VOLTAGE"] = "The current voltage of the actuator";
    m_mAttributeDictionary["CURRENT_SCALE"] = "The physical upper limit of output current";
    m_mAttributeDictionary["VELOCITY_SCALE"] = "The physical upper limit of output speed";
    m_mAttributeDictionary["ACTUATOR_TEMPERATURE"] = "The current temperature of the actuator";
    m_mAttributeDictionary["INVERTER_TEMPERATURE"] = "The current temperature of the inverter";
    m_mAttributeDictionary["ACTUATOR_SWITCH"] = "The actuator is enabled";
    m_mAttributeDictionary["FIRMWARE_VERSION"] = "The actuator's firmware version, newer versions tend to have more "
                                                 "features";
    m_mAttributeDictionary["ONLINE_STATUS"] = "The actuator can still be discovered by the system";
    m_mAttributeDictionary["SN_ID"] = "The serial number of the actuator";
    m_mAttributeDictionary["ERROR_ID"] = "Errors currently present in the actuator";
    m_mAttributeDictionary["INIT_STATE"] = "The actuator is initialized and ready to go";
    m_mAttributeDictionary["VERSION_430"] = "Version number of 430, for tech support";
    m_mAttributeDictionary["FRENQUENCY_430"] = "Frequency  of 430, for tech support";



}