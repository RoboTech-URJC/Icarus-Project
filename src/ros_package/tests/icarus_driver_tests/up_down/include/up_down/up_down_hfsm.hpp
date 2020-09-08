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

/* Author:Pablo Castellanos p4b5.git@gmail.com */

#ifndef UP_DOWN__HFSM_HPP_
#define UP_DOWN__HFSM_HPP_

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Empty.h>
#include "boca_negra/Bocanegra.hpp"
#include "icarus_driver/IcarusDriver.hpp"

namespace up_down
{
class UpDownHFSM : public boca_negra::Bocanegra, icarus_driver::IcarusDriver
{
public:
	UpDownHFSM();

	void step();

private:
	void initParams();
	void droneStateCb(const mavros_msgs::State::ConstPtr & msg);
	void moverLocalCb(const std_msgs::Empty & msg);

	/* -------------- */

	void initCodeOnce();
	void initCodeIterative(){}

	void armCodeOnce();
	void armCodeIterative(){}

	void takeoffCodeOnce();
	void takeoffCodeIterative();

	void landCodeOnce();
	void landCodeIterative();

	bool init2arm();
	bool arm2takeoff();
	bool takeoff2land();
	bool land2finish();

	static const int INIT = 0;
	static const int ARM = 1;
	static const int TAKEOFF = 2;
	static const int LAND =  3;
	static const int FINISH = 4;

	ros::NodeHandle nh_;
	ros::Subscriber drone_state_sub_, altitude_sub_, mover_local_sub_;

	int state_;
	double target_altitude_, target_x_, target_angle;
	float current_altitude_;
	bool code_once_executed_, mover_local_finished_;
	std::string drone_state_topic_, local_pos_topic_;
	mavros_msgs::State drone_state_;
};
};	// namespace up_down

#endif	// UP_DOWN__HFSM_HPP_
