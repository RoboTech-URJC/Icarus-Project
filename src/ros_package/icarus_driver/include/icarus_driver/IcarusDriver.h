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
    void arm_disarm(int arm);
    void takeoff(float lat, float lon, float alt);

  private:

    enum{
      NumberOfSends = 300,
    };

    void notify_ack(std::string msg); //notify if last command wass succesfully or not

    ros::NodeHandle nh_;
    ros::Publisher ack_notifier_;

  };

};
