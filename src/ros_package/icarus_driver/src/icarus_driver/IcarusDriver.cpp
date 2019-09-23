/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include "icarus_driver/IcarusDriver.h"

namespace icarus_driver
{

Icarus_Driver::Icarus_Driver():
  nh_("~")
{
  ROS_INFO("%s", "Hello World! I'm Icarus Drone");
}

};
