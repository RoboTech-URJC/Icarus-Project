/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include <string>
#include <iterator>

#include "icarus_driver_msgs/state_change.h"
#include "icarus_driver_msgs/state.h"

//Global const
std::vector<icarus_driver_msgs::state> v;


bool
change_state(icarus_driver_msgs::state_change::Request  &req,
            	icarus_driver_msgs::state_change::Response &res)
{
	bool finish = false;
	int i = 0;
	while(! finish && i < v.size())
	{
		finish = v.at(i).node_name.data.compare(req.node_name.data) == 0;
	}

	icarus_driver_msgs::state s;
	if(finish){
		v.at(i).is_active = req.active;
	}else{
		s.node_name = req.node_name;
		s.is_active = req.active;
		v.push_back(s);
	}

	res.node_name = req.node_name;
	res.active = req.active;

	return true;
}

bool
is_active(icarus_driver_msgs::state_change::Request  &req,
            	icarus_driver_msgs::state_change::Response &res)
{
  std::string str, str_node;

  str_node = req.node_name.data;

  std::vector<icarus_driver_msgs::state>::iterator it;
  for(it = v.begin(); it < v.end(); it++){
    str = it->node_name.data;
    if(str.compare(str_node) == 0){
      break;
    }
  }

  res.node_name = req.node_name;
  res.active = it->is_active;
}

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "states_listener_node");

	ros::NodeHandle nh;
	ros::ServiceServer activate_server_srv_ = nh.advertiseService("change_state", change_state);
  ros::ServiceServer is_active_server_srv_ = nh.advertiseService("is_active", change_state);

	ros::spin();

  return 0;
}
