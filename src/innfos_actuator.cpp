#include "actuatorcontroller_ros.h"
#include "actuatorcontroller.h"
#include <ros/package.h>

ActuatorController_ROS * INNFOS_ptr;


void actuator_event_callback(const ros::TimerEvent& event)
{
  ActuatorController::getInstance()->processEvents();
}




int main(int argc, char **argv) {

  ros::init(argc, argv, "innfos_actuator");

  ros::NodeHandle nh;
  ROS_INFO("Started Actuator ROS Node Ver. 3.1.0");

  ActuatorController::initController();

  Actuator::ErrorsDefine _errorCode;

  ActuatorController::getInstance()->lookupActuators(_errorCode);


  INNFOS_ptr = new ActuatorController_ROS();


  ros::Timer timer1 = nh.createTimer(ros::Duration(0.005), actuator_event_callback);

//  ros::Timer timer3 = nh.createTimer(ros::Duration(0.01), &ActuatorController_ROS::unleaseCallback , INNFOS_ptr);

    while (ros::ok())
    {


        INNFOS_ptr->releaseJointStates();
        INNFOS_ptr->updateROSParam();

        ros::spinOnce();

//        loop_rate.sleep();
//        ++count;

        ros::Duration(0.01).sleep();
    }

//  ros::spin();

  return 0;
}