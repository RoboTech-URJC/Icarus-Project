#include <ros/ros.h>
#include "icarus_driver/IcarusDriver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trial_Node");
  icarus_driver::Icarus_Driver icarus;

  icarus.set_mode("STABILIZE");
  icarus.arm_disarm(1);
  icarus.takeoff(0.0, 0.0, 5.0);

  return 0;
}
