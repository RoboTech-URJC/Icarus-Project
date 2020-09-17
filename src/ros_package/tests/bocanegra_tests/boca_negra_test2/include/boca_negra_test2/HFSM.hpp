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

#ifndef BOCA_NEGRA_TEST2__HFSM_HPP_
#define BOCA_NEGRA_TEST2__HFSM_HPP_

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include "boca_negra/Bocanegra.h"

namespace boca_negra_test2
{

class HFSM : public boca_negra::Bocanegra
{
public:
	HFSM();

	void step();

private:
	void initParams();
	void droneStateCb(const mavros_msgs::State::ConstPtr & msg);
	void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr & msg);

	/* -------------- */

	void initCodeOnce();
	void initCodeIterative(){}

	void setModeTakeoffCodeOnce();
	void setModeTakeoffCodeIterative();

	void armCodeOnce();
	void armCodeIterative();

	void setModeLandCodeOnce();
	void setModeLandCodeIterative();

	void disarmCodeOnce();
	void disarmCodeIterative();

	/* -------------- */

	bool init2setModeTakeoff();
	bool setModeTakeoff2arm();
	bool arm2setModeLand();
	bool setModeLand2disarm();
	bool disarm2init();

	static const int INIT = 0;
	static const int SET_MODE_TAKEOFF = 1;
	static const int ARM = 2;
	static const int SET_MODE_LAND = 3;
	static const int DISARM = 4;
	static const int FINISH = 5;

	ros::NodeHandle nh_;
	ros::Publisher set_mode_pub_, arm_disarm_pub_;
	ros::Subscriber drone_state_sub_, local_pose_sub_;

	int state_;
	bool code_once_executed_, is_armed_;
	double altitude_;
	std::string takeoff_mode_, land_mode_, drone_state_topic_, drone_mode_,
		local_pose_topic_;
};

};	//namespace boca_negra_test2

#endif	// BOCA_NEGRA_TEST2__HFSM_HPP_
