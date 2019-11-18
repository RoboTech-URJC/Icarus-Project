/*********************************************************************
*  Software License Agreement (BSD License)
*********************************************************************/

/* Author: Fernando Gonzalez fergonzaramos@yahoo.es  */

#include <ros/ros.h>

#include <string>

namespace boca_negra
{
	class Bocanegra
	{

	public:
		Bocanegra();

		void activate(std::string node_name);
		void deactivate(std::string node_name);

	private:

		ros::NodeHandle nh_;
		ros::ServiceClient client_state_service;

	};
};
