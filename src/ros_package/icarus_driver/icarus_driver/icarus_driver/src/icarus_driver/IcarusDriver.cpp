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

#include "icarus_driver/IcarusDriver.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <string>
#include "tf/transform_datatypes.h"
#include "icarus_driver_msgs/TargetPose.h"

namespace icarus_driver
{

IcarusDriver::IcarusDriver():
  nh_("~")
{
  ROS_INFO("%s\n", "SETTING UP ICARUS DRONE...");

  initParams();

  ack_notifier_ = nh_.advertise<std_msgs::String>("/icarus_driver/ack_notify", 1);
  drone_state_sub_ = nh_.subscribe(state_topic_, 1, &IcarusDriver::droneStateCb, this);
  battery_status_sub_ = nh_.subscribe(
    battery_status_topic_, 1, &IcarusDriver::batteryStatusCb, this);

  ROS_INFO("%s\n", " *** ICARUS DRONE OPERATING *** ");
}

StatusType
IcarusDriver::getStatus()
{
  /*
   * Returns Icarus's current status
   */

  return icarus_status_;
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

  if (sc.call(setmode))
  {
  msg = "a";
  ROS_INFO("%s\n", "Set mode Succesfully");
  }
  else
  {
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

  if (arm != 0 && arm != 1)
  {
    ROS_ERROR("%s\n", "ERROR ARMING PARAM");
    return;
  }
  // call to ros service
  ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandBool>(arm_disarm_srv_);

  mavros_msgs::CommandBool arming;

  std::string s;
  if (arm == 1)
  {
    s = "Armed";
    arming.request.value = true;
  }
  else
  {
    s = "Disarmed";
    arming.request.value = false;
  }

  std::string msg;
  if (sc.call(arming))
  {
    msg = "c";
    ROS_INFO("%s %s\n", s.c_str(), "Succesfully");
  }
  else
  {
    msg = "b";
    ROS_ERROR("%s %s\n", s.c_str(), "Failed");
  }
  notifyAck(msg);
}

void
IcarusDriver::takeoff(double alt)
{
  /*
   * params: altitude to takeoff
   */

  ros::ServiceClient sc = nh_.serviceClient<icarus_driver_msgs::TargetPose>(
    "/icarus_driver/mover_local_srv");

  icarus_driver_msgs::TargetPose take_off;
  geometry_msgs::PoseStamped target_pose;

  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.0;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = alt;

  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 1.0;

  take_off.request.target_pose = target_pose;

  if (sc.call(take_off))
  {
    ROS_INFO("%s", "Take Off Succesfully");
  }
  else
  {
    ROS_ERROR("%s", "Take Off Failed");
  }
}

void
IcarusDriver::land()
{
  /*
   * params: latitude, longitude and altitude to takeoff
   */

  ros::ServiceClient sc = nh_.serviceClient<mavros_msgs::CommandTOL>(land_srv_);

  mavros_msgs::CommandTOL srv;
  srv.request.altitude = 0.0;

  if (sc.call(srv))
  {
    ROS_INFO("%s", "Land Succesfully");
  }
  else
  {
    ROS_ERROR("%s", "Land Failed, your drone could be in danger");
  }
}


void
IcarusDriver::moveLocalTo(double x, double y, double z)
{
  /*
   * params: latitude, longitude and altitude to move to a specific point
   */

  ros::ServiceClient sc = nh_.serviceClient<icarus_driver_msgs::TargetPose>(
    "/icarus_driver/mover_local_srv");

  icarus_driver_msgs::TargetPose move_to;
  geometry_msgs::PoseStamped target_pose;

  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;

  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 1.0;

  move_to.request.target_pose = target_pose;

  if (sc.call(move_to))
  {
    ROS_INFO("%s", "Move To Pose Succesfully");
  }
  else
  {
    ROS_ERROR("%s", "Move To Pose Failed");
  }
}

void
IcarusDriver::turnLocalTo(double roll, double pitch, double yaw)
{
  /*
   * params: roll, pitch and yaw to turn
   */

  ros::ServiceClient sc = nh_.serviceClient<icarus_driver_msgs::TargetPose>(
    "/icarus_driver/mover_local_srv");

  icarus_driver_msgs::TargetPose move_to;
  geometry_msgs::PoseStamped target_pose;

  target_pose.header.frame_id = "base_link";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.0;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.0;

  // Create Quaternion from RPY angle

  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
    roll, pitch, yaw);

  move_to.request.target_pose = target_pose;

  if (sc.call(move_to))
  {
    ROS_INFO("%s", "Move To Pose Succesfully");
  }
  else
  {
    ROS_ERROR("%s", "Move To Pose Failed");
  }
}

// PRIVATE METHODS

void
IcarusDriver::droneStateCb(const mavros_msgs::State::ConstPtr & msg)
{
  icarus_status_.is_armed = msg->armed;
}


void
IcarusDriver::batteryStatusCb(const sensor_msgs::BatteryState::ConstPtr & msg)
{
  icarus_status_.battery_percentage = msg->percentage;
}

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
  local_pose_topic_ = "/mavros/local_position/pose";
  local_pose_setter_topic_ = "/mavros/setpoint_position/local";
  land_srv_ = "/mavros/cmd/land";
  state_topic_ = "/mavros/state";
  battery_status_topic_ = "/mavros/battery";


  nh_.param("set_mode_srv", set_mode_srv_, set_mode_srv_);
  nh_.param("arm_disarm_srv", arm_disarm_srv_, arm_disarm_srv_);
  nh_.param("local_pose_topic", local_pose_topic_, local_pose_topic_);
  nh_.param("local_pose_setter_topic", local_pose_setter_topic_, local_pose_setter_topic_);
  nh_.param("land_srv", land_srv_, land_srv_);
  nh_.param("state_topic_", state_topic_, state_topic_);
  nh_.param("battery_status_topic_", battery_status_topic_, battery_status_topic_);
}
};  // namespace icarus_driver
