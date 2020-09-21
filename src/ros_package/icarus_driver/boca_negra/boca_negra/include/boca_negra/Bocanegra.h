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

#ifndef BOCA_NEGRA_BOCANEGRA_H
#define BOCA_NEGRA_BOCANEGRA_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "boca_negra_msgs/states.h"
#include "boca_negra_msgs/state.h"

namespace boca_negra
{
class Bocanegra
{
public:
  Bocanegra();

protected:
  void activate(std::string node_name);
  void deactivate(std::string node_name);

  bool isActive();

private:
  void statesCallback(const boca_negra_msgs::states::ConstPtr msg);

  bool isActive(std::string node_name);

  ros::NodeHandle nh_;

  ros::ServiceClient activate_client_service_;
  ros::Subscriber states_sub_;

  std::vector<boca_negra_msgs::state>v_;
  std::string this_node_;
};
};  // namespace boca_negra

#endif  // BOCA_NEGRA_BOCANEGRA_H
