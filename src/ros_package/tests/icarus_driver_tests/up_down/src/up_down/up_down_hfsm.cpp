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

#include "up_down/up_down_hfsm.hpp"

namespace up_down
{
UpDownHFSM::UpDownHFSM()
: icarus_driver::IcarusDriver(),
	nh_(""),
	state_(INIT),
	code_once_executed_(false),
	mover_local_finished_(false)
{
	target_altitude_ = 5.0;
	target_x_ = 3.0;
	mover_local_sub_ = nh_.subscribe("/icarus_driver/mover_local/finished", 1, &UpDownHFSM::moverLocalCb, this);
}

void
UpDownHFSM::step()
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
UpDownHFSM::moverLocalCb(const std_msgs::Empty & msg)
{
	mover_local_finished_ = true;
}

void
UpDownHFSM::initCodeOnce()
{
	ROS_WARN("State [%s]\n", "Init");
	setMode("OFFBOARD");

}

void
UpDownHFSM::armCodeOnce()
{
	ROS_WARN("State [%s]\n", "Arm");
	armDisarm(1);

}

void
UpDownHFSM::takeoffCodeOnce()
{
	ROS_WARN("State [%s]\n", "Takeoff");
	activate("mover_local_node");
	takeoff(target_altitude_);
	setMode("OFFBOARD");
}

void
UpDownHFSM::takeoffCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Takeoff");

}

void
UpDownHFSM::landCodeOnce()
{
	ROS_WARN("State [%s]\n", "Land");
	land();
}

void
UpDownHFSM::landCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Land");
}


bool
UpDownHFSM::init2arm()
{
	return true;
}

bool
UpDownHFSM::arm2takeoff()
{
	return getStatus().is_armed;
}

bool
UpDownHFSM::takeoff2land()
{
	if (mover_local_finished_) {
		mover_local_finished_ = false;
		return true;
	}
	return false;
}


bool
UpDownHFSM::land2finish()
{
	return true;
}

};	// namespace simple_mover_drone
