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
#include <string>
#include <std_msgs/String.h>
#include "icarus_driver/IcarusDriver.h"
#include "boca_negra/Bocanegra.h"

#define HZ 5

class SetMode : public boca_negra::Bocanegra
{
public:
  SetMode()
  :
  boca_negra::Bocanegra(),
  nh_("~"),
  set_mode_topic_("/icarus_driver/set_mode")
  {
    current_mode_ = "";
    new_mode_ = "";

    set_mode_sub_ = nh_.subscribe(set_mode_topic_, 1, &SetMode::setModeCb, this);
  }

  void
  update()
  {
    if (isActive())
    {
      if (current_mode_ == new_mode_)
        return;

      current_mode_ = new_mode_;
      icarus_.setMode(new_mode_);
    }
    else
    {
      current_mode_ = "";
      new_mode_ = "";
    }
  }

private:
  void setModeCb(const std_msgs::String::ConstPtr & msg)
  {
  new_mode_ = msg->data;
  }

  ros::NodeHandle nh_;

  std::string set_mode_topic_, current_mode_, new_mode_;
  icarus_driver::IcarusDriver icarus_;
  ros::Subscriber set_mode_sub_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "set_mode_node");
  SetMode set_mode;

  ros::Rate rate = HZ;
  while (ros::ok())
  {
    set_mode.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
