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

#ifndef ICARUS_DRIVER__ICARUS_DRIVER_HPP_
#define ICARUS_DRIVER__ICARUS_DRIVER_HPP_

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/Altitude.h>

namespace icarus_driver
{
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

	//revise inline functions in case code style conflicts
	float getBatteryPercentage(){return battery_percentage_;};
	bool getArmStatus(){return is_armed_;};
	float getLocalAltitude(){return local_altitude_;};



  private:
    void initParams();
    void notifyAck(std::string msg);  // notify if last command wass succesfully or not
	void isArmedCb(const mavros_msgs::State::ConstPtr& msg);
	void batteryStatusCb(const sensor_msgs::BatteryState::ConstPtr& msg);
	void localAltitudeCb(const mavros_msgs::Altitude::ConstPtr& msg);

    ros::NodeHandle nh_;

    ros::Publisher ack_notifier_;
	ros::Subscriber is_armed_sub_;
	ros::Subscriber battery_status_sub_;
	ros::Subscriber local_altitude_info_sub_;

	bool is_armed_;
	float battery_percentage_, local_altitude_;

  protected:
    std::string set_mode_srv_, arm_disarm_srv_, takeoff_srv_, local_pose_topic_,
      local_pose_setter_topic_, land_srv_, is_armed_topic_, battery_status_topic_, local_altitude_info_topic_;
  };
};  //namespace icarus_driver

#endif  // ICARUS_DRIVER__ICARUS_DRIVER_HPP_
