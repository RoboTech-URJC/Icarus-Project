#include <ros/ros.h>
#include <string>

#include "icarus_driver/IcarusDriver.h"
#include "icarus_driver_msgs/states.h"
#include "icarus_driver_msgs/state.h"

class Executor
{
public:
  Executor():
    nh_("~")
  {

    init_params();

    is_active_ = false;
    flight_mode_ = "STABILIZE";

    st_subscriber_ = nh_.subscribe(st_topic_, 1, &Executor::states_machine_Cb, this);
    mode_subscriber_ = nh_.subscribe(flight_mode_topic_, 1, &Executor::flight_mode_Cb, this);
  }

  void
  update()
  {
    if(is_active_)
    {
      icarus_.set_mode(flight_mode_);
    }
  }

private:

  void
  states_machine_Cb(const icarus_driver_msgs::states::ConstPtr& msg)
  {
    std::vector<icarus_driver_msgs::state> v = msg->array;
    bool finish = false;
    int iterator = 0;
    while(! finish && iterator < v.size())
    {
      if(v[iterator].node_name.data == "set_mode_node")
      {
        finish = true;
      }else{
        iterator++;
      }
    }
    if(finish)
    {
      is_active_ = v[iterator].is_active.data;
    }
  }

  void
  flight_mode_Cb(const std_msgs::String::ConstPtr& msg)
  {
    flight_mode_ = msg->data;
  }

  void
  init_params()
  {
    st_topic_ = "/icarus_driver/states_machine";
    flight_mode_topic_ = "/icarus_driver/flight_mode";

    nh_.param("states_machine_topic", st_topic_, st_topic_);
    nh_.param("flight_mode_topic", flight_mode_topic_, flight_mode_topic_);
  }

  std::string st_topic_, flight_mode_topic_, flight_mode_;
  bool is_active_;
  icarus_driver::Icarus_Driver icarus_;

  ros::Subscriber st_subscriber_, mode_subscriber_;
  ros::NodeHandle nh_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_mode_node");
  Executor executor;

  ros::Rate rate = 10; //10 Hz
  while(ros::ok()){
    executor.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
