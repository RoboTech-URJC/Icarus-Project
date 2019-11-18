/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>

#include "boca_negra/Bocanegra.h"

namespace boca_negra
{

Bocanegra::Bocanegra():
	nh_()
{
	ROS_INFO("%s\n", ros::this_node::getName().c_str());
}

};
