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

#include "icarus_driver/IcarusDriver.hpp"
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/String.h>

namespace icarus_driver
{

Icarus_Driver::Icarus_Driver():
  nh_("~")
{
  ROS_INFO("%s", "Hello World! I'm Icarus Drone");

  ack_notifier_ = nh_.advertise<std_msgs::String>("/icarus_driver/ack_notify", 1);
}

void
Icarus_Driver::set_mode(std::string mode)
{
  /*
   *param mode: flight mode you want to drone change
   */

  // call to ros service

  ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  mavros_msgs::SetMode setmode;
  setmode.request.custom_mode = mode;

  std::string msg;

  if (sc.call(setmode)) {
    msg = "a";
    ROS_INFO("%s\n", "Set mode Succesfully");
  } else {
    msg = "b";
    ROS_INFO("%s\n", "Set mode Failed");
  }

  notify_ack(msg);
}

void
Icarus_Driver::arm_disarm(int arm)
{
  /*
   * param arm: if 1, arm drone, if 0, disarm drone
   */

   if (arm != 0 && arm != 1) {
     ROS_ERROR("%s\n", "ERROR ARMING PARAM");
   } else{
     // call to ros service
     ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandBool>(
       "/mavros/cmd/arming");

     mavros_msgs::CommandBool arming;

     std::string s;
     if (arm == 1) {
       s = "Armed";
       arming.request.value = true;
     } else {
       s = "Disarmed";
       arming.request.value = false;
     }

     std::string msg;
     if (sc.call(arming)) {
       msg = "c";
       ROS_INFO("%s %s\n", s.c_str(), "Succesfully");
     } else {
       msg = "b";
       ROS_ERROR("%s %s\n", s.c_str(), "Failed");
     }

   }
}

void
Icarus_Driver::takeoff(float lat, float lon, float alt)
{
  /*
   * params: latitude, longitude and altitude to takeoff
   */

   ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandTOL>(
     "/mavros/cmd/takeoff");

   mavros_msgs::CommandTOL take_off;

   take_off.request.latitude = lat;
   take_off.request.longitude = lon;
   take_off.request.altitude = alt;

   if (sc.call(take_off)) {
     ROS_INFO("%s", "Take Off Succesfully");
   } else {
     ROS_ERROR("%s","take Off Failed");
   }
}

// PRIVATE METHODS

void
Icarus_Driver::notify_ack(std::string msg)
{
  std_msgs::String m;

  m.data = msg;
  ack_notifier_.publish(m);
}
};  // namespace icarus_driver
