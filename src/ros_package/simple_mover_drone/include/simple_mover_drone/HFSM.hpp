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

#ifndef SIMPLE_MOVER_DRONE__HFSM_HPP_
#define SIMPLE_MOVER_DRONE__HFSM_HPP_

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include "boca_negra/Bocanegra.hpp"
#include "icarus_driver/IcarusDriver.hpp"

namespace simple_mover_drone
{
class HFSM : public boca_negra::Bocanegra, icarus_driver::IcarusDriver
{
public:
	HFSM();

	void step();

private:
	void initParams();
	void droneStateCb(const mavros_msgs::State::ConstPtr & msg);
	void altitudeCb(const mavros_msgs::Altitude::ConstPtr & msg);

	/* -------------- */

	void initCodeOnce();
	void initCodeIterative(){}

	void armCodeOnce();
	void armCodeIterative(){}

	void takeoffCodeOnce();
	void takeoffCodeIterative();

	void forwardCodeOnce();
	void forwardCodeIterative();

	bool init2arm();
	bool arm2takeoff();
	bool takeoff2forward();
	bool forward2turn(){return false;}

	static const int INIT = 0;
	static const int ARM = 1;
	static const int TAKEOFF = 2;
	static const int FORWARD = 3;
	static const int TURN = 3;

	ros::NodeHandle nh_;
	ros::Subscriber drone_state_sub_, altitude_sub_;
	ros::Publisher local_pos_pub_;

	int state_;
	double target_altitude_, target_x_, target_angle;
	float current_altitude_;
	bool code_once_executed_;
	std::string drone_state_topic_, local_pos_topic_, altitude_topic_;
	mavros_msgs::State drone_state_;
};
};	// namespace simple_mover_drone

#endif	// SIMPLE_MOVER_DRONE__HFSM_HPP_
