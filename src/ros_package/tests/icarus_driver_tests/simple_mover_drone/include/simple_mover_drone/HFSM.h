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

#ifndef SIMPLE_MOVER_DRONE_HFSM_H
#define SIMPLE_MOVER_DRONE_HFSM_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "boca_negra/Bocanegra.h"
#include "icarus_driver/IcarusDriver.h"

namespace simple_mover_drone
{
class HFSM : public boca_negra::Bocanegra, icarus_driver::IcarusDriver
{
public:
  HFSM();

  void step();

private:
  void initParams();
  void moverLocalCb(const std_msgs::Empty & msg);

  /* -------------- */

  void initCodeOnce();
  void initCodeIterative()
  {}

  void armCodeOnce();
  void armCodeIterative()
  {}

  void takeoffCodeOnce();
  void takeoffCodeIterative();

  void forwardCodeOnce();
  void forwardCodeIterative();

  void turnCodeOnce();
  void turnCodeIterative();

  void landCodeOnce();
  void landCodeIterative();

  bool init2arm();
  bool arm2takeoff();
  bool takeoff2forward();
  bool forward2turn();
  bool turn2forward();
  bool turn2land();
  bool land2finish();

  static const int INIT = 0;
  static const int ARM = 1;
  static const int TAKEOFF = 2;
  static const int FORWARD = 3;
  static const int TURN = 4;
  static const int LAND = 5;

  ros::NodeHandle nh_;
  ros::Subscriber drone_state_sub_, altitude_sub_, mover_local_sub_;
  ros::Publisher local_pos_pub_;

  int state_, iterations_;
  double target_altitude_, target_x_, target_angle;
  float current_altitude_;
  bool code_once_executed_, mover_local_finished_;
  std::string drone_state_topic_, local_pos_topic_;
  mavros_msgs::State drone_state_;
};
};  // namespace simple_mover_drone

#endif  // SIMPLE_MOVER_DRONE_HFSM_H
