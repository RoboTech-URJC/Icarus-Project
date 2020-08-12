/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include "boca_negra/Bocanegra.hpp"
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "boca_negra_msgs/state_change.h"

namespace boca_negra
{

Bocanegra::Bocanegra()
: nh_("~")
{
	this_node_ = ros::this_node::getName();
	this_node_.erase(0, 1);

	states_sub_ = nh_.subscribe(
		"/boca_negra/states", 1, &Bocanegra::statesCallback, this);

	activate_client_service_ = nh_.serviceClient<boca_negra_msgs::state_change>(
		"/boca_negra/change_state");
}

bool
Bocanegra::isActive()
{
	int i = 0;
	bool finish = false;
	while (!finish && i < v_.size()) {
		finish = v_.at(i).node_name.data.compare(this_node_) == 0;
		if (finish)
			break;
		i++;
	}
	if (!finish)
		return false;

	return v_.at(i).is_active.data;
}
bool
Bocanegra::isActive(std::string node_name)
{
	int i = 0;
	bool finish = false;
	while (!finish && i < v_.size()) {
		finish = v_.at(i).node_name.data.compare(node_name) == 0;
		if (finish)
			break;
		i++;
	}
	if (!finish)
		return false;

	return v_.at(i).is_active.data;
}

void
Bocanegra::activate(std::string node_name)
{
	if (isActive(node_name) )
		return;
	ROS_WARN("Activating [%s] node\n", node_name.c_str());

	std_msgs::String nn;
	std_msgs::Bool b;
	boca_negra_msgs::state_change srv;

	nn.data = node_name;
	b.data = true;

	srv.request.node_name = nn;
	srv.request.active = b;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Service Failed");
	}
}

void
Bocanegra::deactivate(std::string node_name)
{
	if (!isActive(node_name) )
		return;

	ROS_WARN("Deactivating [%s] node\n", node_name.c_str());

	std_msgs::String nn;
	std_msgs::Bool b;
	boca_negra_msgs::state_change srv;

	nn.data = node_name;
	b.data = false;

	srv.request.node_name = nn;
	srv.request.active = b;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Service Failed");
	}
}

void
Bocanegra::statesCallback(const boca_negra_msgs::states::ConstPtr msg)
{
	v_ = msg->array;
}
};	// namespace boca_negra
