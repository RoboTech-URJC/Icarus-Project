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

/* Author: Pablo Castellanos p4b5git@gmail.com */

#include "up_down/up_down.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace up_down
{
UP_DOWN::UP_DOWN()
: icarus_driver::IcarusDriver(),
	nh_(""),
	state_(INIT),
	code_once_executed_(false),
	mover_local_finished_(false)
{
	initParams();
	target_altitude_ = 5.0;
	target_x_ = 3.0;
	drone_state_sub_ = nh_.subscribe(drone_state_topic_, 1, &UP_DOWN::droneStateCb, this);
	mover_local_sub_ = nh_.subscribe("/icarus_driver/mover_local/finished", 1, &UP_DOWN::moverLocalCb, this);
	local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(local_pos_topic_, 1);
}

void
UP_DOWN::step()
{
	switch (state_) {
		case INIT:
			if (!code_once_executed_) {
				initCodeOnce();
				code_once_executed_ = true;
			}
			initCodeIterative();

			if (init2arm()) {
				code_once_executed_ = false;
				state_ = ARM;
			}
			break;

		case ARM:
			if (!code_once_executed_) {
				armCodeOnce();
				code_once_executed_ = true;
			}
			armCodeIterative();

			if (arm2takeoff()) {
				code_once_executed_ = false;
				state_ = TAKEOFF;
			}
			break;

		case TAKEOFF:
			if (!code_once_executed_) {
				takeoffCodeOnce();
				code_once_executed_ = true;
			}
			takeoffCodeIterative();

			if (takeoff2land()) {
				code_once_executed_ = false;
				state_ = LAND;
			}
			break;

		case LAND:
			if (!code_once_executed_) {
				landCodeOnce();
				code_once_executed_ = true;
			}
			landCodeIterative();

			if (land2finish()) {
				code_once_executed_ = false;
				state_ = FINISH;
			}
			break;

		default:
			break;
	}
}

void
UP_DOWN::initParams()
{
	drone_state_topic_ = "/mavros/state";
	local_pos_topic_ = "/mavros/setpoint_position/local";

	nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
	nh_.param("local_pos_topic", local_pos_topic_, local_pos_topic_);
}

void
UP_DOWN::droneStateCb(const mavros_msgs::State::ConstPtr & msg)
{
	drone_state_ = *msg;
}

void
UP_DOWN::moverLocalCb(const std_msgs::Empty & msg)
{
	mover_local_finished_ = true;
}

void
UP_DOWN::initCodeOnce()
{
	ROS_WARN("State [%s]\n", "Init");
	setMode("OFFBOARD");
}

void
UP_DOWN::armCodeOnce()
{
	ROS_WARN("State [%s]\n", "Arm");
	armDisarm(1);
}

void
UP_DOWN::takeoffCodeOnce()
{
	ROS_WARN("State [%s]\n", "Takeoff");
	activate("mover_local_node");
	takeoff(target_altitude_);
	setMode("OFFBOARD");
}

void
UP_DOWN::takeoffCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Takeoff");
}

void
UP_DOWN::landCodeOnce()
{
	ROS_WARN("State [%s]\n", "Land");
	land();
}

void
UP_DOWN::landCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Land");
}


bool
UP_DOWN::init2arm()
{
	return true;
}

bool
UP_DOWN::arm2takeoff()
{
	return drone_state_.armed;
}

bool
UP_DOWN::takeoff2land()
{
	if (mover_local_finished_) {
		mover_local_finished_ = false;
		return true;
	}
	return false;
}


bool
UP_DOWN::land2finish()
{
	return true;
}

};	// namespace simple_mover_drone
