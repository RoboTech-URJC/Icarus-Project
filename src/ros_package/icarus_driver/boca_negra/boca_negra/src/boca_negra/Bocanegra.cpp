/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "boca_negra/Bocanegra.h"
#include "icarus_driver_msgs/state_change.h"

namespace boca_negra
{

Bocanegra::Bocanegra():
	nh_()
{
	this_node_ = ros::this_node::getName();

	activate_client_service_ = nh_.serviceClient<icarus_driver_msgs::state_change>("change_state");
	is_active_client_srv_ = nh_.serviceClient<icarus_driver_msgs::state_change>("is_active");
}

bool
Bocanegra::is_active()
{
	std_msgs::String nn;
	icarus_driver_msgs::state_change srv;

	nn.data = this_node_;
	srv.request.node_name = nn;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Srvice Failed");
	}

	return srv.response.active.data;
}

void
Bocanegra::activate(std::string node_name)
{
	std_msgs::String nn;
	std_msgs::Bool b;
	icarus_driver_msgs::state_change srv;

	nn.data = node_name;
	b.data = true;

	srv.request.node_name = nn;
	srv.request.active = b;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Srvice Failed");
	}
}

void
Bocanegra::deactivate(std::string node_name)
{
	std_msgs::String nn;
	std_msgs::Bool b;
	icarus_driver_msgs::state_change srv;

	nn.data = node_name;
	b.data = false;

	srv.request.node_name = nn;
	srv.request.active = b;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Srvice Failed");
	}
}

};
