/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include "icarus_driver/IcarusDriver.h"
#include "mavros_msgs/SetMode.h"

#include <string>

namespace icarus_driver
{

Icarus_Driver::Icarus_Driver():
  nh_("~")
{
  ROS_INFO("%s", "Hello World! I'm Icarus Drone");
}

void
Icarus_Driver::set_mode(std::string mode)
{
  /*
   *param mode: flight mode you want to drone change
   */

  //call to arming ros service
  ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::SetMode>("Service_Name");

  mavros_msgs::SetMode setmode;
  setmode.request.custom_mode = mode;
  if(sc.call(setmode)){
    ROS_INFO("%s", "Service Call Good");
  }else{
    ROS_INFO("%s", "Service call Bad");
  }

}

void
arm_disarm(int arm){
  
}

};
