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

		bool is_active();

	private:

		ros::NodeHandle nh_;
		ros::ServiceClient activate_client_service_, is_active_client_srv_;

		std::string this_node_;

	};
};
