/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando Gonzalez fergonzaramos@yahoo.es  */

#include <string>
#include <ros/ros.h>
#include "boca_negra_msgs/states.h"
#include "boca_negra_msgs/state.h"

namespace boca_negra
{
	class Bocanegra
	{

	public:
		Bocanegra();

		void activate(std::string node_name);
		void deactivate(std::string node_name);

		bool isActive();

	private:

		void statesCallback(const boca_negra_msgs::states::ConstPtr msg);

		bool isActive(std::string node_name);

		ros::NodeHandle nh_;

		ros::ServiceClient activate_client_service_;
		ros::Subscriber states_sub_;

		std::vector<boca_negra_msgs::state>v_;
		std::string this_node_;

	};
};
