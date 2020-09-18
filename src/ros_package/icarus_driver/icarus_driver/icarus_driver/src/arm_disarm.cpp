// Copyright 2020 Robotech URJC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Fernando González fergonzaramos@yahoo.es */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <string>
#include "icarus_driver/IcarusDriver.h"
#include "boca_negra/Bocanegra.h"

#define HZ 5

class ArmDisarm : public boca_negra::Bocanegra
{
public:
  ArmDisarm()
  :
  nh_("~"),
  have_to_arm_(false),
  drone_is_armed_(false)
  {
    initParams();
    set_mode_sub_ = nh_.subscribe(
      "/icarus_driver/arm_disarm", 1, &ArmDisarm::setModeCb, this);

    drone_state_sub_ = nh_.subscribe(
      drone_state_topic_, 1, &ArmDisarm::droneStateCb, this);
  }

  void
  update()
  {
    int arm_mode;
    if (!isActive())
      return;

    if (drone_is_armed_ != have_to_arm_)
    {
      arm_mode = have_to_arm_;
      icarus_.armDisarm(arm_mode);
    }
  }

private:
  void
  setModeCb(const std_msgs::Bool::ConstPtr & msg)
  {
    have_to_arm_ = msg->data;
  }

  void
  droneStateCb(const mavros_msgs::State::ConstPtr & msg)
  {
    drone_is_armed_ = msg->armed;
  }

  void
  initParams()
  {
    drone_state_topic_ = "/mavros/state";

    nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
  }

  ros::NodeHandle nh_;

  ros::Subscriber set_mode_sub_, drone_state_sub_;
  icarus_driver::IcarusDriver icarus_;

  std::string drone_state_topic_;
  bool drone_is_armed_, have_to_arm_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_disarm_node");

  ArmDisarm arm_disarm;

  ros::Rate loop_rate(HZ);
  while (ros::ok())
  {
    arm_disarm.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
