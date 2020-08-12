/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "icarus_driver/IcarusDriver.h"
#include "boca_negra/Bocanegra.hpp"

#define HZ 5

class Executor : public boca_negra::Bocanegra
{
public:
  Executor()
  : boca_negra::Bocanegra(), nh_("~"), set_mode_topic_("/icarus_driver/set_mode")
  {
    current_mode_ = "";
    new_mode_ = "";

    set_mode_sub_ = nh_.subscribe(set_mode_topic_, 1, &Executor::setModeCb, this);
  }

  void
  update()
  {
    if (isActive()) {
      if (current_mode_ == new_mode_)
        return;

      current_mode_ = new_mode_;
      icarus_.set_mode(new_mode_);
    }else{
      ROS_INFO("NOT Active\n");
    }
  }

private:

  void setModeCb(const std_msgs::String::ConstPtr & msg)
  {
    new_mode_ = msg->data;
  }

  ros::NodeHandle nh_;

  std::string set_mode_topic_, current_mode_, new_mode_;
  icarus_driver::Icarus_Driver icarus_;
  ros::Subscriber set_mode_sub_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "set_mode_node");
  Executor executor;

  ros::Rate rate = HZ;
  while(ros::ok()){
    executor.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
