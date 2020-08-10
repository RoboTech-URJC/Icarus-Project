/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "boca_negra/Bocanegra.h"
#include "boca_negra_msgs/state_change.h"

namespace boca_negra
{

Bocanegra::Bocanegra():
	nh_()
{
	this_node_ = ros::this_node::getName();

	states_sub_ = nh_.subscribe(
		"/boca_negra/states", 1, &Bocanegra::statesCallback, this);

	activate_client_service_ = nh_.serviceClient<boca_negra_msgs::state_change>("change_state");
	// Este servicio hay que quitarlo
	is_active_client_srv_ = nh_.serviceClient<boca_negra_msgs::state_change>("is_active");
}

bool
Bocanegra::is_active()
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

void
Bocanegra::activate(std::string node_name)
{
	std_msgs::String nn;
	std_msgs::Bool b;
	boca_negra_msgs::state_change srv;

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
	boca_negra_msgs::state_change srv;

	nn.data = node_name;
	b.data = false;

	srv.request.node_name = nn;
	srv.request.active = b;
	if(! activate_client_service_.call(srv)){
		ROS_ERROR("%s\n", "Ros Srvice Failed");
	}
}

void
Bocanegra::statesCallback(const boca_negra_msgs::states::ConstPtr msg)
{
	v_ = msg->array;
}

};
