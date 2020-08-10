/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <string>
#include <iterator>
#include <ros/ros.h>
#include "boca_negra_msgs/state_change.h"
#include "boca_negra_msgs/state.h"
#include "boca_negra_msgs/states.h"

#define HZ 5

class StListener
{
public:

  StListener()
  : nh_("~")
  {
    states_pub_ = nh_.advertise<boca_negra_msgs::states>("/boca_negra/states", 1);
    activate_server_srv_ = nh_.advertiseService(
      "/boca_negra/change_state", &StListener::change_state, this);
  }

  void
  update()
  {
    boca_negra_msgs::states states;
    states.array = v_;
    states_pub_.publish(states);
  }

private:

  bool
  change_state(boca_negra_msgs::state_change::Request  &req,
              	boca_negra_msgs::state_change::Response &res)
  {
  	bool finish = false;
  	int i = 0;
  	while (! finish && i < v_.size()) {
  		finish = v_.at(i).node_name.data.compare(req.node_name.data) == 0;
      if (finish)
        break;
      i++;
  	}

  	boca_negra_msgs::state s;
  	if(finish){
  		v_.at(i).is_active = req.active;
  	}else{
  		s.node_name = req.node_name;
  		s.is_active = req.active;
  		v_.push_back(s);
  	}

  	res.node_name = req.node_name;
  	res.active = req.active;

  	return true;
  }

  ros::NodeHandle nh_;

  ros::Publisher states_pub_;
  ros::ServiceServer activate_server_srv_;

  std::vector<boca_negra_msgs::state> v_;

};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "states_listener_node");

  StListener st_listener;

  ros::Rate loop_rate(HZ);
  while (ros::ok()) {
    st_listener.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
