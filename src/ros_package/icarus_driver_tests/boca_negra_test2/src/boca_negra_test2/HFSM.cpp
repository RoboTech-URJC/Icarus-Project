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

#include "boca_negra_test2/HFSM.hpp"
#include <std_msgs/String.h>

namespace boca_negra_test2
{

HFSM::HFSM()
:	boca_negra::Bocanegra(), nh_("~"), state_(INIT), code_once_executed_(false)
{
	ROS_INFO("I'm the States Machine HFSM\n");

	set_mode_pub_ = nh_.advertise<std_msgs::String>("/icarus_driver/set_mode", 1);
}

void
HFSM::step()
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

		default:
			break;
	}
}

void
HFSM::initCodeOnce()
{
	ROS_WARN("%s\n", "HSFM Begins!\n");
}

void
HFSM::setModeTakeoffCodeOnce()
{
	ROS_WARN("state [%s]\n", "Set Mode Takeoff");
	activate("set_mode_node");
}

void
HFSM::setModeTakeoffCodeIterative()
{
	ROS_INFO("[%s] Code Iterative\n", "Set Mode Takeoff");
	std_msgs::String msg;
	msg.data = "AUTO.TAKEOFF";
	set_mode_pub_.publish(msg);
}

};	// namespace boca_negra_test2
