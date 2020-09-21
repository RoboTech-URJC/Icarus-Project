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

#include <string>
#include <vector>
#include <ros/ros.h>
#include "boca_negra_msgs/state_change.h"
#include "boca_negra_msgs/state.h"
#include "boca_negra_msgs/states.h"

#define HZ 5

class StListener
{
public:
  StListener()
  :
  nh_("~")
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
    while (!finish && i < v_.size())
    {
      finish = v_.at(i).node_name.data.compare(req.node_name.data) == 0;
      if (finish)
        break;
      i++;
    }

    boca_negra_msgs::state s;
    if (finish)
    {
      v_.at(i).is_active = req.active;
    }
    else
    {
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
  while (ros::ok())
  {
    st_listener.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
