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

#include "boca_negra_test2/HFSM_using_driver.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define MAXALTITUDE 1.0
#define MINALTITUDE 0.2

namespace boca_negra_test2
{

HFSMDriver::HFSMDriver()
:	boca_negra::Bocanegra(), nh_("~"), state_(INIT), code_once_executed_(false),
	takeoff_mode_("AUTO.TAKEOFF"), land_mode_("AUTO.LAND")
{
	ROS_INFO("I'm the States Machine HFSM\n");

	drone_mode_ = "";
	is_armed_ = false;
	initParams();

	drone_state_sub_ = nh_.subscribe(drone_state_topic_, 1, &HFSMDriver::droneStateCb, this);
	local_pose_sub_ = nh_.subscribe(local_pose_topic_, 1, &HFSMDriver::localPoseCb, this);
}

void
HFSMDriver::step()
{
	switch (state_) {
		case INIT:
			if (!code_once_executed_) {
				initCodeOnce();
				code_once_executed_ = true;
			}
			initCodeIterative();

			if (init2setModeTakeoff()) {
				code_once_executed_ = false;
				state_ = SET_MODE_TAKEOFF;
			}
			break;

		case SET_MODE_TAKEOFF:
			if (!code_once_executed_) {
				setModeTakeoffCodeOnce();
				code_once_executed_ = true;
			}
			setModeTakeoffCodeIterative();

			if (setModeTakeoff2arm()) {
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

			if (arm2setModeLand()) {
				code_once_executed_ = false;
				state_ = SET_MODE_LAND;
			}
			break;

		case SET_MODE_LAND:
			if (!code_once_executed_) {
				setModeLandCodeOnce();
				code_once_executed_ = true;
			}
			setModeLandCodeIterative();

			if (setModeLand2disarm()) {
				code_once_executed_ = false;
				state_ = DISARM;
			}
			break;

		case DISARM:
			if (!code_once_executed_) {
				disarmCodeOnce();
				code_once_executed_ = true;
			}
			disarmCodeIterative();

			if (disarm2init()) {
				code_once_executed_ = false;
				state_ = INIT;
			}
			break;

		default:
			deactivate("arm_disarm_node");
			ROS_WARN("JOB FINISHED!\n");
			break;
	}
}

void
HFSMDriver::droneStateCb(const mavros_msgs::State::ConstPtr & msg)
{
	drone_mode_ = msg->mode;
	is_armed_ = msg->armed;
}

void
HFSMDriver::localPoseCb(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
	altitude_ = msg->pose.position.z;
}

void
HFSMDriver::initParams()
{
	// Get params from launch
	drone_state_topic_ = "/mavros/state";
	local_pose_topic_ = "/mavros/local_position/pose";

	nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
	nh_.param("local_pose_topic", local_pose_topic_, local_pose_topic_);
}

void
HFSMDriver::initCodeOnce()
{
	ROS_WARN("%s\n", "HFSM Begins!\n");
}

void
HFSMDriver::setModeTakeoffCodeOnce()
{
	ROS_WARN("State [%s]\n", "Set Mode Takeoff");
	setMode(takeoff_mode_);
	//activate("set_mode_node");
}

void
HFSMDriver::setModeTakeoffCodeIterative()
{
	ROS_INFO("[%s] Code Iterative\n", "Set Mode Takeoff");
}

void
HFSMDriver::armCodeOnce()
{
	ROS_WARN("State [%s]\n", "Arm");
	armDisarm(1);
}

void
HFSMDriver::armCodeIterative()
{
	ROS_INFO("[%s] Code Iterative\n", "Arm");
}

void
HFSMDriver::setModeLandCodeOnce()
{
	ROS_WARN("State [%s]\n", "Set Mode Land");
	setMode(land_mode_);
}

void
HFSMDriver::setModeLandCodeIterative()
{
	ROS_INFO("[%s] Code Iterative\n", "Set Mode Land");
}

void
HFSMDriver::disarmCodeOnce()
{
	ROS_WARN("State [%s]\n", "Disarm");
	armDisarm(0);
}

void
HFSMDriver::disarmCodeIterative()
{
	ROS_INFO("[%s] Code Iterative\n", "Disarm");
}

bool
HFSMDriver::init2setModeTakeoff()
{
	return true;
}

bool
HFSMDriver::setModeTakeoff2arm()
{
	return drone_mode_ == takeoff_mode_;
}

bool
HFSMDriver::arm2setModeLand()
{
	return altitude_ >= MAXALTITUDE;
}

bool
HFSMDriver::setModeLand2disarm()
{
	return altitude_ <= MINALTITUDE;
}

bool
HFSMDriver::disarm2init()
{
	return !is_armed_;
}

};	// namespace boca_negra_test2
