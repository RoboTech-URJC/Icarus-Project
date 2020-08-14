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

IcarusDriver::IcarusDriver():
  nh_("~")
{
  ROS_INFO("%s", "Hello World! I'm Icarus Drone");

  initParams();
  
  ack_notifier_ = nh_.advertise<std_msgs::String>("/icarus_driver/ack_notify", 1);
}

void
IcarusDriver::setMode(std::string mode)
{
  /*
   *param mode: flight mode you want to drone change
   */

  // call to ros service

  ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_srv_);

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

  notifyAck(msg);
}

void
IcarusDriver::armDisarm(int arm)
{
  /*
   * param arm: if 1, arm drone, if 0, disarm drone
   */

   if (arm != 0 && arm != 1) {
     ROS_ERROR("%s\n", "ERROR ARMING PARAM");
     return;
   }
   // call to ros service
   ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandBool>(arm_disarm_srv_);

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
   notifyAck(msg);
}

void
IcarusDriver::takeoff(float lat, float lon, float alt)
{
  /*
   * params: latitude, longitude and altitude to takeoff
   */

   ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandTOL>(takeoff_srv_);

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
IcarusDriver::notifyAck(std::string msg)
{
  std_msgs::String m;

  m.data = msg;
  ack_notifier_.publish(m);
}

void
IcarusDriver::initParams()
{
  set_mode_srv_ = "/mavros/set_mode";
  arm_disarm_srv_ = "/mavros/cmd/arming";
  takeoff_srv_ = "/mavros/cmd/takeoff";

  nh_.param("set_mode_srv", set_mode_srv_, set_mode_srv_);
  nh_.param("arm_disarm_srv", arm_disarm_srv_, arm_disarm_srv_);
  nh_.param("takeoff_srv", takeoff_srv_, takeoff_srv_);
}

};  // namespace icarus_driver
