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
#include "simple_mover_drone/HFSM.h"

namespace simple_mover_drone
{
HFSM::HFSM()
: icarus_driver::IcarusDriver(),
  nh_(""),
  state_(INIT),
  code_once_executed_(false),
  mover_local_finished_(false)
{
  initParams();
  target_altitude_ = 3.0;
  target_x_ = 3.0;
  iterations_ = 0;
  mover_local_sub_ = nh_.subscribe("/icarus_driver/mover_local/finished", 1, &HFSM::moverLocalCb, this);
}

void
HFSM::step()
{
  switch (state_)
  {
    case INIT:
      if (!code_once_executed_)
      {
        initCodeOnce();
        code_once_executed_ = true;
      }
      initCodeIterative();

      if (init2arm())
      {
        code_once_executed_ = false;
        state_ = ARM;
      }
      break;

      case ARM:
      if (!code_once_executed_)
      {
        armCodeOnce();
        code_once_executed_ = true;
      }
      armCodeIterative();

      if (arm2takeoff())
      {
        code_once_executed_ = false;
        state_ = TAKEOFF;
      }
      break;

    case TAKEOFF:
      if (!code_once_executed_)
      {
        takeoffCodeOnce();
        code_once_executed_ = true;
      }
      takeoffCodeIterative();

      if (takeoff2forward())
      {
        code_once_executed_ = false;
        state_ = FORWARD;
      }
      break;

    case FORWARD:
      if (!code_once_executed_)
      {
        forwardCodeOnce();
        code_once_executed_ = true;
      }
      forwardCodeIterative();

      if (forward2turn())
      {
        code_once_executed_ = false;
        state_ = TURN;
      }
      break;

    case TURN:
      if (!code_once_executed_)
      {
        turnCodeOnce();
        code_once_executed_ = true;
      }
      turnCodeIterative();

      if (turn2land())
      {
        code_once_executed_ = false;
        state_ = LAND;
      }
      else if (turn2forward())
      {
        code_once_executed_ = false;
        state_ = FORWARD;
      }
      break;

    case LAND:
      if (!code_once_executed_)
      {
        landCodeOnce();
        code_once_executed_ = true;
      }
      landCodeIterative();

      if (land2finish())
      {
        code_once_executed_ = false;
        state_ = 6;
      }
      break;

    default:
      deactivate("mover_local_node");
      break;
  }
}

void
HFSM::initParams()
{
  drone_state_topic_ = "/mavros/state";
  local_pos_topic_ = "/mavros/setpoint_position/local";

  nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
  nh_.param("local_pos_topic", local_pos_topic_, local_pos_topic_);
}

void
HFSM::moverLocalCb(const std_msgs::Empty & msg)
{
  mover_local_finished_ = true;
}

void
HFSM::initCodeOnce()
{
  ROS_WARN("State [%s]\n", "Init");
  setMode("OFFBOARD");
}

void
HFSM::armCodeOnce()
{
  ROS_WARN("State [%s]\n", "Arm");
  armDisarm(1);
}

void
HFSM::takeoffCodeOnce()
{
  ROS_WARN("State [%s]\n", "Takeoff");
  activate("mover_local_node");
  takeoff(target_altitude_);
  setMode("OFFBOARD");
}

void
HFSM::takeoffCodeIterative()
{
  ROS_INFO("State [%s] Code Iterative\n", "Takeoff");
}

void
HFSM::forwardCodeOnce()
{
  ROS_WARN("State [%s]\n", "Forward");
  moveLocalTo(target_x_, 0.0, 0.0);
}

void
HFSM::forwardCodeIterative()
{
  ROS_INFO("State [%s] Code Iterative\n", "Forward");
}

void
HFSM::turnCodeOnce()
{
  ROS_WARN("State [%s]\n", "Turn");
  iterations_++;

  // Turn Pi/2:

  turnLocalTo(0.0, 0.0, 3.1416 / 2.0);
}

void
HFSM::turnCodeIterative()
{
  ROS_INFO("State [%s] Code Iterative\n", "Turn");
}

void
HFSM::landCodeOnce()
{
  ROS_WARN("State [%s]\n", "Land");

  // Land:

  land();
}

void
HFSM::landCodeIterative()
{
  ROS_INFO("State [%s] Code Iterative\n", "Land");
}

bool
HFSM::init2arm()
{
  return true;
}

bool
HFSM::arm2takeoff()
{
  return getStatus().is_armed;
}

bool
HFSM::takeoff2forward()
{
  if (mover_local_finished_)
  {
    mover_local_finished_ = false;
    return true;
  }
  return false;
}

bool
HFSM::forward2turn()
{
  if (mover_local_finished_)
  {
    mover_local_finished_ = false;
    return true;
  }
  return false;
}

bool
HFSM::turn2forward()
{
  if (mover_local_finished_)
  {
  mover_local_finished_ = false;
  return true;
  }
  return false;
}

bool
HFSM::turn2land()
{
  if (mover_local_finished_ && iterations_ == 4)
  {
    mover_local_finished_ = false;
    return true;
  }
  return false;
}

bool
HFSM::land2finish()
{
  return true;
}

};  // namespace simple_mover_drone
