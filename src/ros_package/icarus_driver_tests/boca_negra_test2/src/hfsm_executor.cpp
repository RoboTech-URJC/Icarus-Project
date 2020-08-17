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
#include "boca_negra_test2/HFSM.hpp"

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "hfsm_executor_node");

	boca_negra_test2::HFSM hfsm_executor;

	ros::Rate loop_rate(5);
	while (ros::ok()) {
		hfsm_executor.step();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
