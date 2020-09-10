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

#ifndef ICARUS_DRIVER_ICARUSDRIVER_H
#define ICARUS_DRIVER_ICARUSDRIVER_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <string>

namespace icarus_driver
{

typedef struct StatusType StatusType;
struct StatusType
{
  bool is_armed;
  float battery_percentage;
};

class IcarusDriver
{
public:
  IcarusDriver();

  void setMode(std::string mode);
  void armDisarm(int arm);
  void takeoff(double alt);
  void land();
  void moveLocalTo(double x, double y, double z);
  void turnLocalTo(double roll, double pitch, double yaw);

  StatusType getStatus();

private:
  void initParams();
  void notifyAck(std::string msg);
  void droneStateCb(const mavros_msgs::State::ConstPtr & msg);
  void batteryStatusCb(const sensor_msgs::BatteryState::ConstPtr & msg);

  ros::NodeHandle nh_;

  ros::Publisher ack_notifier_;
  ros::Subscriber drone_state_sub_;
  ros::Subscriber battery_status_sub_;
  ros::Subscriber local_altitude_info_sub_;

  StatusType icarus_status_;

protected:
  std::string set_mode_srv_, arm_disarm_srv_, takeoff_srv_, local_pose_topic_,
    local_pose_setter_topic_, land_srv_, state_topic_, battery_status_topic_;
};
};  // namespace icarus_driver

#endif  // ICARUS_DRIVER_ICARUSDRIVER_H
