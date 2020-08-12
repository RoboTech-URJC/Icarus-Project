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

#include <ros/ros.h>
#include "boca_negra/Bocanegra.hpp"

namespace boca_negra_test2
{

class HFSM : public boca_negra::Bocanegra
{
public:

	HFSM();

	void step();

private:

	void initCodeOnce();
	void initCodeIterative(){}

	void armCodeOnce(){}
	void armCodeIterative(){}

	void disarmCodeOnce(){}
	void disarmCodeIterative(){}

	void setModeTakeoffCodeOnce();
	void setModeTakeoffCodeIterative();

	void setModeLandCodeOnce(){}
	void setModeLandCodeIterative(){}

	/* -------------- */

	bool init2setModeTakeoff(){return true;}
	bool setModeTakeoff2arm(){return true;}
	bool arm2setModeLand(){return true;}
	bool setModeLand2disarm(){return true;}

	static const int INIT = 0;
	static const int SET_MODE_TAKEOFF = 1;
	static const int ARM = 2;
	static const int SET_MODE_LAND = 3;
	static const int DISARM = 4;

	ros::NodeHandle nh_;
	ros::Publisher set_mode_pub_;

	int state_;
	bool code_once_executed_;

};
};	//namespace boca_negra_test2

#endif	// BOCA_NEGRA_TEST2__HFSM_HPP_
