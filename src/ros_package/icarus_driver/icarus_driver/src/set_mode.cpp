#include <ros/ros.h>
#include <string>
#include <std_msgs/Empty.h>

#include "icarus_driver/IcarusDriver.h"
#include "boca_negra_msgs/states.h"
#include "boca_negra_msgs/state.h"
#include "mavros_msgs/State.h"

class Executor
{
public:
  Executor():
    nh_("~")
  {

    init_params();

    is_active_ = false;
    flight_mode_ = "";

    st_subscriber_ = nh_.subscribe(st_topic_, 1, &Executor::states_machine_Cb, this);
    mode_subscriber_ = nh_.subscribe(flight_mode_topic_, 1, &Executor::flight_mode_Cb, this);
    drone_state_subscriber_ = nh_.subscribe(drone_state_topic_, 1, &Executor::drone_state_Cb, this);

    is_finished_publisher_ = nh_.advertise<std_msgs::Empty>(is_finished_topic_, 1);
  }

  void
  update()
  {
    if(is_active_)
    {
      ROS_INFO("Active");
      if(flight_mode_ == "")
        return;
      icarus_.set_mode(flight_mode_);
    }else{
      flight_mode_ = "";
    }
  }

private:

  void
  drone_state_Cb(const mavros_msgs::State::ConstPtr& msg)
  {
    if(is_active_)
    {
      std_msgs::Empty m;
      if(msg->mode == flight_mode_)
      {
        is_finished_publisher_.publish(m);
      }
    }
  }

  void
  states_machine_Cb(const boca_negra_msgs::states::ConstPtr& msg)
  {
    std::vector<boca_negra_msgs::state> v = msg->array;
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
    if(is_active_)
    {
      flight_mode_ = msg->data;
    }
  }

  void
  init_params()
  {
    st_topic_ = "/icarus_driver/states_machine";
    flight_mode_topic_ = "/icarus_driver/flight_mode";
    is_finished_topic_ = "/icarus_driver/set_mode/is_finished";
    drone_state_topic_ = "/mavros/state";

    nh_.param("states_machine_topic", st_topic_, st_topic_);
    nh_.param("flight_mode_topic", flight_mode_topic_, flight_mode_topic_);
    nh_.param("is_finished_topic", is_finished_topic_, is_finished_topic_);
    nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
  }

  std::string st_topic_, flight_mode_topic_, flight_mode_, is_finished_topic_,
                drone_state_topic_;
  bool is_active_;
  icarus_driver::Icarus_Driver icarus_;

  ros::Subscriber st_subscriber_, mode_subscriber_, drone_state_subscriber_;
  ros::Publisher is_finished_publisher_;
  ros::NodeHandle nh_;

};

int
main(int argc, char **argv)
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
