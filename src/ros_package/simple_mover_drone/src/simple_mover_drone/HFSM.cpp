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

#include "simple_mover_drone/HFSM.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace simple_mover_drone
{
HFSM::HFSM()
: icarus_driver::IcarusDriver(),
	nh_(""),
	state_(INIT),
	code_once_executed_(false)
{
	initParams();
	target_altitude_ = 3.0;
	target_x_ = 5.0;
	drone_state_sub_ = nh_.subscribe(drone_state_topic_, 1, &HFSM::droneStateCb, this);
	altitude_sub_ = nh_.subscribe(altitude_topic_, 1, &HFSM::altitudeCb, this);
	local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(local_pos_topic_, 1);
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

			if (takeoff2forward()) {
				code_once_executed_ = false;
				state_ = FORWARD;
			}
			break;

		case FORWARD:
			if (!code_once_executed_) {
				forwardCodeOnce();
				code_once_executed_ = true;
			}
			forwardCodeIterative();

			if (forward2turn()) {
				code_once_executed_ = false;
				state_ = TURN;
			}
			break;

		default:
			break;
	}
}

void
HFSM::initParams()
{
	drone_state_topic_ = "/mavros/state";
	local_pos_topic_ = "/mavros/setpoint_position/local";
	altitude_topic_ = "/mavros/altitude";

	nh_.param("drone_state_topic", drone_state_topic_, drone_state_topic_);
	nh_.param("local_pos_topic", local_pos_topic_, local_pos_topic_);
	nh_.param("altitude_topic", altitude_topic_, altitude_topic_);
}

void
HFSM::droneStateCb(const mavros_msgs::State::ConstPtr & msg)
{
	drone_state_ = *msg;
}

void
HFSM::altitudeCb(const mavros_msgs::Altitude::ConstPtr & msg)
{
	current_altitude_ = msg ->local;
}

void
HFSM::initCodeOnce()
{
	ROS_WARN("State [%s]\n", "Init");
}

void
HFSM::armCodeOnce()
{
	ROS_WARN("State [%s]\n", "Arm");
	armDisarm(1);
}

void
HFSM::takeoffCodeOnce()
{
	ROS_WARN("State [%s]\n", "Takeoff");
	setMode("OFFBOARD");
}

void
HFSM::takeoffCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Takeoff");

	geometry_msgs::PoseStamped tgt_pose;
	tgt_pose.header.stamp = ros::Time::now();
	tgt_pose.header.frame_id = "base_link";

	tgt_pose.pose.position.z = target_altitude_;

	local_pos_pub_.publish(tgt_pose);
}

void
HFSM::forwardCodeOnce()
{
	ROS_WARN("State [%s]\n", "Forward");
}

void
HFSM::forwardCodeIterative()
{
	ROS_INFO("State [%s] Code Iterative\n", "Forward");

	geometry_msgs::PoseStamped tgt_pose;
	tgt_pose.header.stamp = ros::Time::now();
	tgt_pose.header.frame_id = "base_link";

	tgt_pose.pose.position.x = target_x_;
	tgt_pose.pose.position.z = target_altitude_;

	local_pos_pub_.publish(tgt_pose);
}

bool
HFSM::init2arm()
{
	return true;
}

bool
HFSM::arm2takeoff()
{
	return drone_state_.armed;
}

bool
HFSM::takeoff2forward()
{
	return abs(current_altitude_ - target_altitude_) < 0.15;
}
};	// namespace simple_mover_drone
