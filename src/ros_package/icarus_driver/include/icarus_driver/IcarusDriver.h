/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando Gonzalez fergonzaramos@yahoo.es  */
#include <ros/ros.h>

#include <string>

namespace icarus_driver
{
  class Icarus_Driver
  {

  public:
    Icarus_Driver();

    void set_mode(std::string mode);

  private:

    ros::NodeHandle nh_;

  };

};
