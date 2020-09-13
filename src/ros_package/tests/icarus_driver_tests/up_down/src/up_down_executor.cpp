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

#include <ros/ros.h>
#include "up_down/up_down_hfsm.hpp"

#define HZ 5

int
main(int argc, char ** argv)
{
	ros::init(argc, argv, "up_down_node");

	up_down::UpDownHFSM up_down;

	ros::Rate rate(HZ);
	while (ros::ok()) {
		up_down.step();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
